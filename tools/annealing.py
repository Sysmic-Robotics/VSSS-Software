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
import queue
import random
import re
import shutil
import signal
import socket
import struct
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
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
    frames_lost_pct: float = 0.0   # peor de los dos engines: frames de visión sin tick

    def fitness(self) -> float:
        return (
            (self.goals_for - self.goals_against)
            + 0.05 * (self.possession_pct - 50.0)
            - 1.0 * self.area_fouls
            - 2.0 * self.retentions
        )

    def as_dict(self) -> dict:
        return dict(self.__dict__, fitness=round(self.fitness(), 3))


def _pb_varint(n: int) -> bytes:
    out = b""
    while True:
        b, n = n & 0x7F, n >> 7
        if not n:
            return out + bytes([b])
        out += bytes([b | 0x80])


def _pb_parse(buf: bytes) -> list[tuple[int, int, object]]:
    """Campos (número, tipo de alambre, valor) de un mensaje protobuf, sin esquema."""
    i, out = 0, []

    def read_varint() -> int:
        nonlocal i
        v = shift = 0
        while True:
            b = buf[i]; i += 1
            v |= (b & 0x7F) << shift; shift += 7
            if not b & 0x80:
                return v

    while i < len(buf):
        tag = read_varint()
        field, wt = tag >> 3, tag & 7
        if wt == 0:
            out.append((field, wt, read_varint()))
        elif wt == 1:
            out.append((field, wt, struct.unpack("<d", buf[i:i + 8])[0])); i += 8
        elif wt == 2:
            n = read_varint(); out.append((field, wt, buf[i:i + n])); i += n
        elif wt == 5:
            out.append((field, wt, buf[i:i + 4])); i += 4
        else:
            raise ValueError(f"wire type {wt}")
    return out


def vision_ball(env: bytes) -> tuple[float, float] | None:
    """Posición de la pelota en un paquete Environment de FIRASim (frame=2 → ball=1 → x=1, y=2)."""
    for f, wt, v in _pb_parse(env):
        if f == 2 and wt == 2:
            for f2, wt2, v2 in _pb_parse(v):
                if f2 == 1 and wt2 == 2:
                    d = {f3: v3 for f3, _, v3 in _pb_parse(v2)}
                    return float(d.get(1, 0.0)), float(d.get(2, 0.0))
    return None


def ball_replacement(x: float, y: float) -> bytes:
    """Packet{replace=2: Replacement{ball=1: BallReplacement{x=1, y=2}}}."""
    ball = b"\x09" + struct.pack("<d", x) + b"\x11" + struct.pack("<d", y)
    replacement = b"\x0a" + _pb_varint(len(ball)) + ball
    return b"\x12" + _pb_varint(len(replacement)) + replacement


class Simulator:
    """FIRASim headless acelerado, con puertos propios, vivo durante toda la corrida."""

    def __init__(self, fps: int, vision_port: int, cmd_port: int, log: Path):
        self.fps = fps
        self.vision_port = vision_port
        self.cmd_port = cmd_port
        self.log = open(log, "a", encoding="utf-8")
        self.proc: subprocess.Popen | None = None
        self.restarts = 0

    def start(self) -> None:
        # Los puertos solo se pasan si no son los de fábrica. (El parche de FIRASim en
        # tools/firasim_cli.patch corrige el segfault al recrear el socket de comandos.)
        args = [str(FIRASIM), "--headless", "--fps", str(self.fps)]
        if self.vision_port != 10002:
            args += ["--vision-port", str(self.vision_port)]
        if self.cmd_port != 20011:
            args += ["--cmd-port", str(self.cmd_port)]
        for intento in range(3):
            self.proc = subprocess.Popen(args, stdout=self.log, stderr=subprocess.STDOUT, cwd=REPO)
            time.sleep(6.0)
            if self.proc.poll() is not None:
                raise RuntimeError(f"FIRASim (visión {self.vision_port}, comandos {self.cmd_port}) terminó al arrancar")
            if self.probe():
                return
            # Instancia 'congelada' (emite visión pero ignora los teleports): se relanza.
            print(f"[annealing] FIRASim en {self.vision_port}/{self.cmd_port} arrancó congelado; se relanza "
                  f"({intento + 1}/3)", flush=True)
            self.stop()
            self.restarts += 1
        raise RuntimeError(f"FIRASim (visión {self.vision_port}, comandos {self.cmd_port}) arranca congelado")

    def probe(self, x: float = 0.31, y: float = 0.22) -> bool:
        """Teleporta la pelota y comprueba que la visión lo refleje."""
        rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        try:
            rx.bind(("", self.vision_port))
            rx.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP,
                          struct.pack("4s4s", socket.inet_aton("224.0.0.1"), socket.inet_aton("0.0.0.0")))
            rx.settimeout(0.5)

            def last_ball(seconds: float) -> tuple[float, float] | None:
                ball, t_end = None, time.monotonic() + seconds
                while time.monotonic() < t_end:
                    try:
                        ball = vision_ball(rx.recv(65536)) or ball
                    except socket.timeout:
                        break
                return ball

            if last_ball(0.3) is None:
                return False
            tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            tx.sendto(ball_replacement(x, y), ("127.0.0.1", self.cmd_port))
            tx.close()
            time.sleep(0.2)
            ball = last_ball(0.3)
            return ball is not None and abs(ball[0] - x) < 0.05 and abs(ball[1] - y) < 0.05
        finally:
            rx.close()

    def stop(self) -> None:
        if self.proc and self.proc.poll() is None:
            self.proc.terminate()
            try:
                self.proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.proc.kill()

    def restart(self) -> None:
        """Se ha visto a FIRASim quedarse 'congelado' (emite visión pero ignora comandos y
        teleports) de forma intermitente; ante un partido inválido se relanza la instancia."""
        self.stop()
        self.restarts += 1
        self.start()

    def close(self) -> None:
        self.stop()
        self.log.close()


@dataclass
class Rival:
    name: str
    coach: str                 # "rule_based" | "heuristic"
    params_file: Path | None   # VSSL_PARAMS del amarillo


FRAME_CLOCK_LINE = re.compile(r"reloj por frames: frames (\d+), ticks (\d+)")


def frames_lost_pct(log: Path) -> float:
    """Frames de visión que el engine no alcanzó a procesar, según su último resumen periódico."""
    last = None
    try:
        for line in log.read_text(encoding="utf-8", errors="replace").splitlines():
            m = FRAME_CLOCK_LINE.search(line)
            if m:
                last = m
    except OSError:
        return 0.0
    if last is None:
        return 0.0
    frames, ticks = int(last.group(1)), int(last.group(2))
    return 100.0 * max(frames - ticks, 0) / max(frames, 1)


class MatchRunner:
    def __init__(self, sim: Simulator, minutes: float, goals: int, referee_port: int, keep_work: bool = False,
                 penalties: int = 0):
        self.sim = sim
        self.minutes = minutes
        self.goals = goals
        # > 0: en vez de un partido, una tanda de penales alternados (match_director --penalties).
        self.penalties = penalties
        self.referee_addr = f"224.5.23.2:{referee_port}"
        self.keep_work = keep_work

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

    def play(self, candidate_file: Path, rival: Rival, seed: int, kickoff: str, tag: str,
             attempt: int = 0) -> MatchResult:
        work = WORK / tag
        shutil.rmtree(work, ignore_errors=True)
        work.mkdir(parents=True, exist_ok=True)
        blue_csv, yellow_csv = work / "azul.csv", work / "amarillo.csv"
        t0 = time.monotonic()
        blue = self._engine("blue", BLUE_SIDE, "heuristic", candidate_file, 1000 + seed, work / "azul.log", blue_csv)
        yellow = self._engine("yellow", YELLOW_SIDE, rival.coach, rival.params_file, 2000 + seed,
                              work / "amarillo.log", yellow_csv)
        time.sleep(4.0)
        out = work / "partido.jsonl"
        limit = 60 + self.minutes * 60 * 2
        try:
            director = subprocess.run(
                [str(DIRECTOR), "--minutes", str(self.minutes), "--goals", str(self.goals), "--kickoff", kickoff,
                 "--speed", str(max(1, self.sim.fps // 60)), "--out", str(out),
                 "--vision-port", str(self.sim.vision_port), "--cmd-port", str(self.sim.cmd_port)]
                + (["--penalties", str(self.penalties)] if self.penalties else []),
                env=dict(os.environ, VSSL_REFEREE_ADDR=self.referee_addr), cwd=REPO,
                stdout=open(work / "director.log", "w"), stderr=subprocess.STDOUT,
                timeout=limit,
            )
            director_code = director.returncode
        except subprocess.TimeoutExpired:
            # `run` ya mató al director; el partido se trata como inválido (abajo).
            director_code = None
        for proc in (blue, yellow):
            proc.terminate()
        for proc in (blue, yellow):
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
        problem = None
        if director_code is None:
            problem = f"el director no terminó en {limit:.0f} s (simulador colgado)"
        elif director_code != 0 or not out.exists():
            raise RuntimeError(f"el director falló en {tag} (código {director_code})")
        else:
            score = json.loads(out.read_text(encoding="utf-8").strip().splitlines()[-1])
            metrics = analyze(str(blue_csv))
            covered = float(metrics.get("duracion_s", 0.0))
            touches = int(metrics.get("toques_propios", 0)) + int(metrics.get("toques_rival", 0))
            if covered < 0.7 * float(score["seconds"]):
                problem = f"el registro azul cubre {covered:.0f} s de {score['seconds']:.0f} s"
            elif touches == 0 and float(metrics.get("pelota_libre_pct", 0.0)) > 99.0:
                problem = "nadie tocó la pelota en todo el partido (simulador congelado)"
        if problem is not None:
            # Partido inválido: el directorio queda para inspección, se relanza el FIRASim de este
            # carril y el partido se repite una vez.
            failed = WORK / f"{tag}_fallido{attempt}"
            shutil.rmtree(failed, ignore_errors=True)
            shutil.move(work, failed)
            print(f"[annealing]   {tag}: {problem} (guardado en {failed})"
                  f"{'; se relanza el simulador y se repite' if attempt < 1 else ''}")
            if attempt < 1:
                self.sim.restart()
                return self.play(candidate_file, rival, seed, kickoff, tag, attempt + 1)
            raise RuntimeError(f"partido {tag} inválido dos veces seguidas")
        result = MatchResult(
            rival=rival.name,
            goals_for=int(score["blue"]), goals_against=int(score["yellow"]),
            possession_pct=float(metrics.get("posesion_propia_pct", 50.0)),
            area_fouls=int(metrics.get("faltas_area_propia_episodios", 0)),
            retentions=int(metrics.get("retencion_10s_episodios", 0)),
            free_balls=int(score.get("free_balls", 0)),
            seconds=float(score["seconds"]), wall_seconds=time.monotonic() - t0,
            frames_lost_pct=max(frames_lost_pct(work / "azul.log"), frames_lost_pct(work / "amarillo.log")),
        )
        if not self.keep_work:
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

    def mean_fitness(self, per_round: int = 3) -> float:
        """Aptitud por tanda (el titular acumula tandas al reevaluarse)."""
        return self.fitness * per_round / max(len(self.matches), 1)

    def as_dict(self) -> dict:
        return {
            "eval": self.index, "params": self.params, "fitness": round(self.fitness, 3),
            "temperature": round(self.temperature, 4), "accepted": self.accepted, "note": self.note,
            "matches": [m.as_dict() for m in self.matches],
        }


@dataclass
class Annealer:
    ranges: dict[str, tuple[float, float]]
    runners: "queue.Queue[MatchRunner]"
    pool: ThreadPoolExecutor
    out: Path
    rng: random.Random
    t0: float
    cooling: float
    reevaluate: bool
    # Veces que se repite la tanda de tres rivales por evaluación (con otras semillas).
    rounds: int = 1
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
        candidate_file = write_params_file(self.out / f"candidato_{index}.json", params)
        rivals = self.rivals() * self.rounds

        def play(k: int, rival: Rival) -> MatchResult:
            kickoff = "blue" if (index + k) % 2 == 0 else "yellow"
            runner = self.runners.get()
            try:
                return runner.play(candidate_file, rival, index * 10 + k, kickoff, tag=f"e{index}_{k}")
            finally:
                self.runners.put(runner)

        # Los partidos de una evaluación se reparten entre los simuladores disponibles.
        matches = list(self.pool.map(lambda kr: play(*kr), enumerate(rivals)))
        for rival, result in zip(rivals, matches):
            print(f"[annealing]   eval {index} vs {rival.name:>12}: {result.goals_for}-{result.goals_against}, "
                  f"posesión {result.possession_pct:.0f} %, faltas {result.area_fouls}, "
                  f"aptitud {result.fitness():+.2f} ({result.wall_seconds:.0f} s, "
                  f"frames perdidos {result.frames_lost_pct:.1f} %)")
        candidate_file.unlink(missing_ok=True)
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
            delta = candidate.fitness - self.incumbent.mean_fitness(3 * self.rounds)
            if delta > 0 and self.reevaluate:
                # El titular se defiende con partidos nuevos: no cede el puesto por suerte.
                self.record(candidate)
                fresh = self.evaluate(len(self.history), self.incumbent.params, temperature, "reevaluación del titular")
                self.record(fresh)
                self.incumbent.matches.extend(fresh.matches)
                delta = candidate.fitness - self.incumbent.mean_fitness(3 * self.rounds)
                candidate.note = f"titular reevaluado: media {self.incumbent.mean_fitness(3 * self.rounds):+.2f}"
                candidate.accepted = delta > 0
                self.rewrite_history()
            else:
                candidate.accepted = delta > 0 or self.rng.random() < math.exp(delta / max(temperature, 1e-9))
                self.record(candidate)
            print(f"[annealing] eval {index}: aptitud {candidate.fitness:+.2f} vs titular "
                  f"{self.incumbent.mean_fitness(3 * self.rounds):+.2f} (Δ {delta:+.2f}, T {temperature:.3f}) → "
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

def report(out: Path) -> int:
    """Tabla del historial: una fila por evaluación, con los parámetros que cambió."""
    path = out / "history.jsonl"
    if not path.exists():
        print(f"[annealing] no hay historial en {path}", file=sys.stderr)
        return 1
    rows = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    ranges = load_ranges()
    defaults = load_defaults(ranges)
    incumbent = None
    print(f"{'eval':>4} {'aptitud':>8} {'T':>6} {'estado':>10}  partidos (gf-gc vs rival)  cambios respecto del titular")
    for r in rows:
        base = incumbent["params"] if incumbent else defaults
        changes = ", ".join(f"{k} {base[k]:.3g}→{r['params'][k]:.3g}" for k in sorted(ranges) if abs(r["params"][k] - base[k]) > 1e-9)
        matches = " ".join(f"{m['goals_for']}-{m['goals_against']}@{m['rival'][:8]}" for m in r["matches"])
        state = "titular" if r["accepted"] else ("reeval" if "reevaluación" in r.get("note", "") else "rechazado")
        print(f"{r['eval']:>4} {r['fitness']:>+8.2f} {r['temperature']:>6.3f} {state:>10}  {matches:<32} {changes or '-'}")
        if r["accepted"]:
            incumbent = r
    best = max(rows, key=lambda r: r["fitness"])
    print(f"\nmejor: eval {best['eval']} con aptitud {best['fitness']:+.2f}; titular final: eval {incumbent['eval'] if incumbent else '-'}")
    return 0


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
    ap.add_argument("--rounds", type=int, default=1, help="tandas de tres rivales por evaluación (menos ruido, más partidos)")
    ap.add_argument("--sims", type=int, default=1, help="simuladores en paralelo (cada uno con sus puertos)")
    ap.add_argument("--vision-port", type=int, default=10002, help="puerto de visión del primer simulador")
    ap.add_argument("--cmd-port", type=int, default=20011, help="puerto de comandos del primer simulador")
    ap.add_argument("--referee-port", type=int, default=10003, help="puerto del árbitro del primer simulador")
    ap.add_argument("--keep-work", action="store_true", help="conservar CSV y logs de cada partido en WORK (depuración)")
    ap.add_argument("--report", action="store_true", help="solo imprimir el historial de --out y salir")
    args = ap.parse_args()

    if args.report:
        return report(args.out)
    for exe in (ENGINE, DIRECTOR, FIRASIM):
        if not exe.exists():
            print(f"[annealing] falta {exe}", file=sys.stderr)
            return 2
    args.out.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    ranges = load_ranges()
    # Un simulador por "carril", con puertos separados de a 10 (visión, comandos y árbitro).
    sims = [Simulator(args.fps, args.vision_port + 10 * k, args.cmd_port + 10 * k, args.out / f"firasim_{k}.log")
            for k in range(max(1, args.sims))]
    runners: "queue.Queue[MatchRunner]" = queue.Queue()
    for k, sim in enumerate(sims):
        runners.put(MatchRunner(sim, args.minutes, args.goals, args.referee_port + k, keep_work=args.keep_work))
    pool = ThreadPoolExecutor(max_workers=len(sims))
    annealer = Annealer(ranges, runners, pool, args.out, random.Random(args.seed), args.t0, args.cooling,
                        not args.no_reevaluate, rounds=max(1, args.rounds))
    annealer.resume()

    def stop_all() -> None:
        for sim in sims:
            sim.close()
        restarts = sum(sim.restarts for sim in sims)
        if restarts:
            print(f"[annealing] simuladores relanzados por partidos inválidos: {restarts}")

    def stop(_sig, _frame):
        print("[annealing] interrumpido; el historial queda y se reanuda con la misma --out")
        stop_all()
        sys.exit(130)

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)
    # La instancia con puertos de fábrica va al final: las demás enlazan 20011 un instante al
    # construirse y eso dejaba 'congelada' a una instancia de fábrica ya en marcha.
    for sim in reversed(sims):
        sim.start()
    print(f"[annealing] {len(sims)} simulador(es) a {args.fps} fps, {3 * max(1, args.rounds)} partidos por evaluación")
    started = time.monotonic()
    try:
        annealer.run(args.evals, load_defaults(ranges))
    finally:
        stop_all()
        pool.shutdown(wait=False)
    print(f"[annealing] {time.monotonic() - started:.0f} s de pared en total")
    return 0


if __name__ == "__main__":
    sys.exit(main())
