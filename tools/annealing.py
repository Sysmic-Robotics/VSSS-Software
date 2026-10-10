#!/usr/bin/env python3
"""Simulated annealing sobre los parámetros de estilo de juego del coach.

Busca, en los rangos de `config/style_ranges.json`, el vector de parámetros con mejor
aptitud en partidos acelerados de FIRASim. Cada candidato juega como AZUL contra tres
rivales (rule_based, la configuración titular y una configuración anterior al azar) y su
aptitud es la diferencia de goles acumulada con desempates por posesión y castigos por
faltas. Un vecino perturba uno a tres parámetros; uno peor se acepta con probabilidad
exp(-Δ/T) y la temperatura baja geométricamente. Cuando un candidato supera al titular,
el titular se vuelve a evaluar antes de ceder el puesto (la evaluación es ruidosa).

Corre desde WSL, con el engine compilado en /root/vsss-target y FIRASim parcheado
(`--fps`, `--vision-port`, `--cmd-port`):

    python3 tools/annealing.py --evals 20 --out logs/annealing/corrida1
    python3 tools/annealing.py --evals 40 --out logs/annealing/corrida1   # reanuda

Salidas en --out: history.jsonl (una línea por evaluación, reanudable), best.json (el
mejor vector como JSON parcial de `coach`, listo para VSSL_PARAMS) y resumen.txt.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import random
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from match_metrics import analyze  # noqa: E402

REPO = Path(__file__).resolve().parent.parent
RANGES_PATH = REPO / "config" / "style_ranges.json"
INTEGER_PARAMS = {"role_min_hold", "skill_min_hold"}

ENGINE = Path("/root/vsss-target/release/rustengine")
DIRECTOR = Path("/root/vsss-target/release/match_director")
FIRASIM = Path("/root/FIRASim/bin/FIRASim")
# Los CSV y logs por tick van a ext4: sobre /mnt/d el lazo salta frames.
WORK = Path("/root/annealing_work")

# Lado en que FIRASim coloca al azul al arrancar (el director lee los lados de las posiciones).
BLUE_SIDE, YELLOW_SIDE = "right", "left"


# ─────────────────────────────────────────────────────────────────────────────
#  Parámetros y vecinos
# ─────────────────────────────────────────────────────────────────────────────

def load_ranges() -> dict[str, tuple[float, float]]:
    data = json.loads(RANGES_PATH.read_text(encoding="utf-8"))
    return {k: (float(lo), float(hi)) for k, (lo, hi) in data.items()}


def load_defaults(ranges: dict[str, tuple[float, float]]) -> dict[str, float]:
    """Valores vigentes de los parámetros de estilo según config/team_params.json."""
    coach = json.loads((REPO / "config" / "team_params.json").read_text(encoding="utf-8-sig"))["coach"]
    return {k: float(coach[k]) for k in ranges}


def clip(key: str, value: float, ranges: dict[str, tuple[float, float]]) -> float:
    lo, hi = ranges[key]
    value = min(max(value, lo), hi)
    return float(round(value)) if key in INTEGER_PARAMS else value


def neighbor(params: dict[str, float], ranges: dict[str, tuple[float, float]], rng: random.Random,
             step_frac: float = 0.10) -> dict[str, float]:
    """Perturba entre 1 y 3 parámetros con un paso gaussiano del 10 % del rango."""
    out = dict(params)
    keys = rng.sample(sorted(ranges), k=rng.randint(1, 3))
    for key in keys:
        lo, hi = ranges[key]
        for _ in range(10):
            candidate = clip(key, out[key] + rng.gauss(0.0, step_frac * (hi - lo)), ranges)
            if candidate != out[key]:
                break
        out[key] = candidate
    return out


def write_params_file(path: Path, params: dict[str, float]) -> Path:
    """JSON parcial de `coach` que el engine carga con VSSL_PARAMS."""
    coach = {k: (int(v) if k in INTEGER_PARAMS else round(v, 4)) for k, v in params.items()}
    path.write_text(json.dumps({"coach": coach}, indent=2), encoding="utf-8")
    return path


# ─────────────────────────────────────────────────────────────────────────────
#  Partidos
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class MatchResult:
    rival: str
    goals_for: int
    goals_against: int
    possession_pct: float
    area_fouls: int
    retentions: int
    free_balls: int
    seconds: float
    wall_seconds: float

    def fitness(self) -> float:
        return (
            (self.goals_for - self.goals_against)
            + 0.05 * (self.possession_pct - 50.0)
            - 1.0 * self.area_fouls
            - 2.0 * self.retentions
        )

    def as_dict(self) -> dict:
        return dict(self.__dict__, fitness=round(self.fitness(), 3))


class Simulator:
    """FIRASim headless acelerado, con puertos propios, vivo durante toda la corrida."""

    def __init__(self, fps: int, vision_port: int, cmd_port: int, log: Path):
        self.fps = fps
        self.vision_port = vision_port
        self.cmd_port = cmd_port
        self.log = open(log, "a", encoding="utf-8")
        self.proc: subprocess.Popen | None = None

    def start(self) -> None:
        # Los puertos solo se pasan si no son los de fábrica: reabrir el socket sobre el
        # mismo puerto deja a FIRASim sin emitir visión.
        args = [str(FIRASIM), "--headless", "--fps", str(self.fps)]
        if self.vision_port != 10002:
            args += ["--vision-port", str(self.vision_port)]
        if self.cmd_port != 20011:
            args += ["--cmd-port", str(self.cmd_port)]
        self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, cwd=REPO)
        time.sleep(6.0)

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()
        self.log.close()


@dataclass
class Rival:
    name: str
    coach: str                 # "rule_based" | "heuristic"
    params_file: Path | None   # VSSL_PARAMS del amarillo (solo heuristic)


class MatchRunner:
    def __init__(self, sim: Simulator, minutes: float, goals: int, referee_port: int):
        self.sim = sim
        self.minutes = minutes
        self.goals = goals
        self.referee_addr = f"224.5.23.2:{referee_port}"

    def _engine(self, color: str, side: str, coach: str, params_file: Path | None, seed: int,
                log: Path, csv: Path) -> subprocess.Popen:
        env = dict(os.environ)
        env.update({
            "VSSL_TEAM_COLOR": color, "VSSL_SIDE": side, "VSSL_COACH": coach,
            "VSSL_BIDIRECTIONAL": "1", "VSSL_VISION_NOISE": "1", "VSSL_FRAME_CLOCK": "1",
            "VSSL_NOISE_SEED": str(seed), "VSSL_MATCH_LOG": str(csv),
            "VSSL_REFEREE_ADDR": self.referee_addr,
            "VSSL_VISION_PORT": str(self.sim.vision_port), "VSSL_CMD_PORT": str(self.sim.cmd_port),
        })
        if params_file is not None:
            env["VSSL_PARAMS"] = str(params_file)
        else:
            env.pop("VSSL_PARAMS", None)
        return subprocess.Popen([str(ENGINE)], env=env, cwd=REPO, stdout=open(log, "w"), stderr=subprocess.STDOUT)

    def play(self, candidate_file: Path, rival: Rival, seed: int, kickoff: str, tag: str) -> MatchResult:
        work = WORK / tag
        work.mkdir(parents=True, exist_ok=True)
        blue_csv, yellow_csv = work / "azul.csv", work / "amarillo.csv"
        t0 = time.monotonic()
        blue = self._engine("blue", BLUE_SIDE, "heuristic", candidate_file, 1000 + seed, work / "azul.log", blue_csv)
        yellow = self._engine("yellow", YELLOW_SIDE, rival.coach, rival.params_file, 2000 + seed,
                              work / "amarillo.log", yellow_csv)
        time.sleep(4.0)
        out = work / "partido.jsonl"
        director = subprocess.run(
            [str(DIRECTOR), "--minutes", str(self.minutes), "--goals", str(self.goals), "--kickoff", kickoff,
             "--speed", str(max(1, self.sim.fps // 60)), "--out", str(out),
             "--vision-port", str(self.sim.vision_port), "--cmd-port", str(self.sim.cmd_port)],
            env=dict(os.environ, VSSL_REFEREE_ADDR=self.referee_addr), cwd=REPO,
            stdout=open(work / "director.log", "w"), stderr=subprocess.STDOUT,
            timeout=60 + self.minutes * 60 * 2,
        )
        for proc in (blue, yellow):
            proc.terminate()
        for proc in (blue, yellow):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        if director.returncode != 0 or not out.exists():
            raise RuntimeError(f"el director falló en {tag} (código {director.returncode})")
        score = json.loads(out.read_text(encoding="utf-8").strip().splitlines()[-1])
        metrics = analyze(str(blue_csv))
        result = MatchResult(
            rival=rival.name,
            goals_for=int(score["blue"]), goals_against=int(score["yellow"]),
            possession_pct=float(metrics.get("posesion_propia_pct", 50.0)),
            area_fouls=int(metrics.get("faltas_area_propia_episodios", 0)),
            retentions=int(metrics.get("retencion_10s_episodios", 0)),
            free_balls=int(score.get("free_balls", 0)),
            seconds=float(score["seconds"]), wall_seconds=time.monotonic() - t0,
        )
        shutil.rmtree(work, ignore_errors=True)
        return result


# ─────────────────────────────────────────────────────────────────────────────
#  Recocido
# ─────────────────────────────────────────────────────────────────────────────

@dataclass
class Evaluation:
    index: int
    params: dict[str, float]
    matches: list[MatchResult]
    temperature: float
    accepted: bool = False
    note: str = ""

    @property
    def fitness(self) -> float:
        return sum(m.fitness() for m in self.matches)

    def mean_fitness(self) -> float:
        """Aptitud por tanda de tres partidos (el titular acumula tandas al reevaluarse)."""
        return self.fitness * 3.0 / max(len(self.matches), 1)

    def as_dict(self) -> dict:
        return {
            "eval": self.index, "params": self.params, "fitness": round(self.fitness, 3),
            "temperature": round(self.temperature, 4), "accepted": self.accepted, "note": self.note,
            "matches": [m.as_dict() for m in self.matches],
        }


@dataclass
class Annealer:
    ranges: dict[str, tuple[float, float]]
    runner: MatchRunner
    out: Path
    rng: random.Random
    t0: float
    cooling: float
    reevaluate: bool
    history: list[Evaluation] = field(default_factory=list)
    incumbent: Evaluation | None = None
    best: Evaluation | None = None

    # ── Persistencia ────────────────────────────────────────────────────────
    def resume(self) -> None:
        path = self.out / "history.jsonl"
        if not path.exists():
            return
        for line in path.read_text(encoding="utf-8").splitlines():
            if not line.strip():
                continue
            d = json.loads(line)
            ev = Evaluation(
                index=d["eval"], params=d["params"], temperature=d["temperature"],
                accepted=d["accepted"], note=d.get("note", ""),
                matches=[MatchResult(**{k: v for k, v in m.items() if k != "fitness"}) for m in d["matches"]],
            )
            self.history.append(ev)
            if ev.accepted:
                self.incumbent = ev
            if self.best is None or ev.fitness > self.best.fitness:
                self.best = ev
        print(f"[annealing] reanudo: {len(self.history)} evaluaciones, titular {self.incumbent.fitness:+.2f}"
              if self.incumbent else "[annealing] historial sin titular")

    def record(self, ev: Evaluation) -> None:
        self.history.append(ev)
        with open(self.out / "history.jsonl", "a", encoding="utf-8") as f:
            f.write(json.dumps(ev.as_dict()) + "\n")
        if self.best is None or ev.fitness > self.best.fitness:
            self.best = ev
            write_params_file(self.out / "best.json", ev.params)

    # ── Evaluación ──────────────────────────────────────────────────────────
    def rivals(self) -> list[Rival]:
        rivals = [Rival("rule_based", "rule_based", None)]
        if self.incumbent is not None:
            rivals.append(Rival("titular", "heuristic", write_params_file(self.out / "titular.json", self.incumbent.params)))
        previous = [ev for ev in self.history if ev is not self.incumbent]
        if previous:
            pick = self.rng.choice(previous)
            rivals.append(Rival(f"anterior_{pick.index}", "heuristic",
                                write_params_file(self.out / "anterior.json", pick.params)))
        while len(rivals) < 3:
            rivals.append(Rival("rule_based", "rule_based", None))
        return rivals

    def evaluate(self, index: int, params: dict[str, float], temperature: float, note: str) -> Evaluation:
        candidate_file = write_params_file(self.out / "candidato.json", params)
        matches = []
        for k, rival in enumerate(self.rivals()):
            kickoff = "blue" if (index + k) % 2 == 0 else "yellow"
            seed = index * 10 + k
            result = self.runner.play(candidate_file, rival, seed, kickoff, tag=f"e{index}_{k}")
            matches.append(result)
            print(f"[annealing]   eval {index} vs {rival.name:>12}: {result.goals_for}-{result.goals_against}, "
                  f"posesión {result.possession_pct:.0f} %, faltas {result.area_fouls}, "
                  f"aptitud {result.fitness():+.2f} ({result.wall_seconds:.0f} s)")
        return Evaluation(index=index, params=params, matches=matches, temperature=temperature, note=note)

    # ── Bucle principal ─────────────────────────────────────────────────────
    def run(self, evals: int, start: dict[str, float]) -> None:
        if self.incumbent is None:
            ev = self.evaluate(len(self.history), start, self.t0, "punto de partida")
            ev.accepted = True
            self.incumbent = ev
            self.record(ev)
            print(f"[annealing] punto de partida: aptitud {ev.fitness:+.2f}")
        while len(self.history) < evals:
            index = len(self.history)
            temperature = self.t0 * self.cooling ** index
            candidate = self.evaluate(index, neighbor(self.incumbent.params, self.ranges, self.rng), temperature, "")
            delta = candidate.fitness - self.incumbent.mean_fitness()
            if delta > 0 and self.reevaluate:
                # El titular se defiende con partidos nuevos: no cede el puesto por suerte.
                self.record(candidate)
                fresh = self.evaluate(len(self.history), self.incumbent.params, temperature, "reevaluación del titular")
                self.record(fresh)
                self.incumbent.matches.extend(fresh.matches)
                delta = candidate.fitness - self.incumbent.mean_fitness()
                candidate.note = f"titular reevaluado: media {self.incumbent.mean_fitness():+.2f}"
                candidate.accepted = delta > 0
                self.rewrite_history()
            else:
                candidate.accepted = delta > 0 or self.rng.random() < math.exp(delta / max(temperature, 1e-9))
                self.record(candidate)
            print(f"[annealing] eval {index}: aptitud {candidate.fitness:+.2f} vs titular "
                  f"{self.incumbent.mean_fitness():+.2f} (Δ {delta:+.2f}, T {temperature:.3f}) → "
                  f"{'ACEPTADO' if candidate.accepted else 'rechazado'}")
            if candidate.accepted:
                self.incumbent = candidate
        self.summary()

    def rewrite_history(self) -> None:
        """Reescribe el JSONL cuando cambia una entrada ya grabada (veredicto tras reevaluar)."""
        with open(self.out / "history.jsonl", "w", encoding="utf-8") as f:
            for ev in self.history:
                f.write(json.dumps(ev.as_dict()) + "\n")

    def summary(self) -> None:
        best = self.best
        if best is None:
            return
        defaults = load_defaults(self.ranges)
        lines = [f"evaluaciones: {len(self.history)}", f"mejor aptitud: {best.fitness:+.2f} (eval {best.index})", ""]
        for k in sorted(self.ranges):
            lines.append(f"  {k:30s} {defaults[k]:8.3f} → {best.params[k]:8.3f}")
        text = "\n".join(lines)
        (self.out / "resumen.txt").write_text(text + "\n", encoding="utf-8")
        print("[annealing] " + text.replace("\n", "\n[annealing] "))


# ─────────────────────────────────────────────────────────────────────────────
#  CLI
# ─────────────────────────────────────────────────────────────────────────────

def main() -> int:
    ap = argparse.ArgumentParser(description="simulated annealing de los parámetros de estilo")
    ap.add_argument("--evals", type=int, default=20, help="evaluaciones totales (incluye las ya hechas)")
    ap.add_argument("--out", type=Path, required=True, help="carpeta de salida (reanuda si existe)")
    ap.add_argument("--fps", type=int, default=480, help="cuadros por segundo de FIRASim (60 = tiempo real)")
    ap.add_argument("--minutes", type=float, default=3.0)
    ap.add_argument("--goals", type=int, default=3)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--t0", type=float, default=1.0 / math.log(2.0), help="temperatura inicial (Δ=1 gol → p=0.5)")
    ap.add_argument("--cooling", type=float, default=0.95)
    ap.add_argument("--no-reevaluate", action="store_true", help="no reevaluar al titular cuando lo superan")
    ap.add_argument("--vision-port", type=int, default=10002)
    ap.add_argument("--cmd-port", type=int, default=20011)
    ap.add_argument("--referee-port", type=int, default=10003)
    args = ap.parse_args()

    for exe in (ENGINE, DIRECTOR, FIRASIM):
        if not exe.exists():
            print(f"[annealing] falta {exe}", file=sys.stderr)
            return 2
    args.out.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    ranges = load_ranges()
    sim = Simulator(args.fps, args.vision_port, args.cmd_port, args.out / "firasim.log")
    runner = MatchRunner(sim, args.minutes, args.goals, args.referee_port)
    annealer = Annealer(ranges, runner, args.out, random.Random(args.seed), args.t0, args.cooling,
                        not args.no_reevaluate)
    annealer.resume()

    def stop(_sig, _frame):
        print("[annealing] interrumpido; el historial queda y se reanuda con la misma --out")
        sim.stop()
        sys.exit(130)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    sim.start()
    started = time.monotonic()
    try:
        annealer.run(args.evals, load_defaults(ranges))
    finally:
        sim.stop()
    print(f"[annealing] {time.monotonic() - started:.0f} s de pared en total")
    return 0


if __name__ == "__main__":
    sys.exit(main())
