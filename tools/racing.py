#!/usr/bin/env python3
"""Búsqueda por coordenadas con aceptación por *racing* sobre pocos parámetros de estilo.

Parte del titular (config/team_params.json o `--start archivo.json`). Para cada parámetro
de la grilla y cada valor, el candidato (titular con ese valor) juega contra el titular por
rondas: en cada ronda, cara a cara en los dos lados de cancha y, como control, candidato y
titular contra rule_based, todo con la misma semilla. Se juegan rondas hasta que la
diferencia de goles cara a cara supere `--z` errores estándar (o se agote `--max-rounds`).
El candidato reemplaza al titular si gana con esa confianza y no empeora con la misma
confianza contra rule_based. Varias barridas, porque el óptimo de un parámetro puede moverse
al cambiar otro. Reanudable: comparaciones terminadas en `comparaciones.jsonl`.

    python3 tools/racing.py --sims 3 --out logs/annealing/racing1 \
        --grid mark_distance=0.25,0.35,0.40 --grid spin_engage_radius=0.32,0.40 \
        --grid score_shot_blocked_factor=0.8,1.0 --grid clear_zone_full_x=0.68,0.75
    python3 tools/racing.py --report --out logs/annealing/racing1
"""
from __future__ import annotations

import argparse
import json
import math
import queue
import signal
import statistics
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from annealing import (  # noqa: E402
    INTEGER_PARAMS, WORK, MatchResult, MatchRunner, Rival, Simulator, load_defaults, load_ranges,
    write_params_file,
)


def mean_sem(values: list[float]) -> tuple[float, float]:
    if len(values) < 2:
        return (values[0] if values else 0.0), float("inf")
    return statistics.mean(values), statistics.stdev(values) / math.sqrt(len(values))


def parse_grid(items: list[str], ranges: dict[str, tuple[float, float]]) -> list[tuple[str, list[float]]]:
    grid = []
    for item in items:
        name, _, values = item.partition("=")
        if name not in ranges:
            sys.exit(f"[racing] {name} no está en config/style_ranges.json")
        lo, hi = ranges[name]
        vals = []
        for v in values.split(","):
            x = float(v)
            if not lo - 1e-9 <= x <= hi + 1e-9:
                sys.exit(f"[racing] {name}={x} fuera del rango [{lo}, {hi}]")
            vals.append(float(round(x)) if name in INTEGER_PARAMS else x)
        grid.append((name, vals))
    return grid


class Racer:
    def __init__(self, runners: "queue.Queue[MatchRunner]", pool: ThreadPoolExecutor, out: Path,
                 min_rounds: int, max_rounds: int, z: float):
        self.runners, self.pool, self.out = runners, pool, out
        self.min_rounds, self.max_rounds, self.z = min_rounds, max_rounds, z
        self.log = open(out / "partidos.jsonl", "a", encoding="utf-8")
        self.comparisons = open(out / "comparaciones.jsonl", "a", encoding="utf-8")

    def _play(self, blue_file: Path, rival: Rival, seed: int, kickoff: str, tag: str) -> MatchResult:
        runner = self.runners.get()
        try:
            return runner.play(blue_file, rival, seed, kickoff, tag=tag)
        finally:
            self.runners.put(runner)

    def race(self, label: str, candidate: dict[str, float], incumbent: dict[str, float], seed0: int) -> dict:
        """Rondas hasta decidir. Devuelve el resumen de la comparación (con `accepted`)."""
        cand_file = write_params_file(self.out / "candidato.json", candidate)
        inc_file = write_params_file(self.out / "titular.json", incumbent)
        rb = Rival("rule_based", "rule_based", None)
        h2h: list[float] = []      # goles candidato − titular por partido
        rb_diff: list[float] = []  # (gf−gc candidato) − (gf−gc titular) vs rule_based, pareado
        wins = draws = losses = 0
        verdict = "indeciso"
        for rnd in range(self.max_rounds):
            seed = seed0 + rnd
            kickoff = "blue" if rnd % 2 == 0 else "yellow"
            jobs = [
                ("c_vs_t", cand_file, Rival("titular", "heuristic", inc_file)),
                ("t_vs_c", inc_file, Rival("candidato", "heuristic", cand_file)),
                ("c_rb", cand_file, rb),
                ("t_rb", inc_file, rb),
            ]
            results = list(self.pool.map(
                lambda j: (j[0], self._play(j[1], j[2], seed, kickoff, tag=f"{label}_r{rnd}_{j[0]}")), jobs))
            r = dict(results)
            for key, res in results:
                self.log.write(json.dumps({"comparacion": label, "ronda": rnd, "partido": key,
                                           **res.as_dict()}) + "\n")
            self.log.flush()
            a, b = r["c_vs_t"], r["t_vs_c"]
            for diff in (a.goals_for - a.goals_against, b.goals_against - b.goals_for):
                h2h.append(float(diff))
                wins += diff > 0; draws += diff == 0; losses += diff < 0
            rb_diff.append(float((r["c_rb"].goals_for - r["c_rb"].goals_against)
                                 - (r["t_rb"].goals_for - r["t_rb"].goals_against)))
            m, s = mean_sem(h2h)
            rm, rs = mean_sem(rb_diff)
            print(f"[racing]   {label} ronda {rnd + 1}: cara a cara {a.goals_for}-{a.goals_against} y "
                  f"{b.goals_against}-{b.goals_for} → acumulado {m:+.2f} ± {s:.2f} ({wins}-{draws}-{losses}); "
                  f"vs rule_based {rm:+.2f} ± {rs:.2f}", flush=True)
            if rnd + 1 >= self.min_rounds and s > 0 and abs(m) >= self.z * s:
                verdict = "mejor" if m > 0 else "peor"
                break
            if rnd + 1 >= self.min_rounds and s == 0:
                verdict = "igual"
                break
        m, s = mean_sem(h2h)
        rm, rs = mean_sem(rb_diff)
        worse_vs_rb = rs > 0 and rm < 0 and abs(rm) >= self.z * rs
        accepted = verdict == "mejor" and not worse_vs_rb
        summary = {
            "comparacion": label, "candidato": candidate, "rondas": len(rb_diff),
            "h2h": round(m, 3), "h2h_sem": round(s, 3), "wdl": [wins, draws, losses],
            "rb": round(rm, 3), "rb_sem": round(rs, 3), "veredicto": verdict,
            "veto_rule_based": worse_vs_rb, "accepted": accepted,
        }
        self.comparisons.write(json.dumps(summary) + "\n"); self.comparisons.flush()
        print(f"[racing] {label}: {verdict}{' pero peor vs rule_based' if worse_vs_rb else ''} → "
              f"{'ACEPTADO' if accepted else 'rechazado'}", flush=True)
        return summary

    def close(self) -> None:
        self.log.close(); self.comparisons.close()


def load_done(out: Path) -> dict[str, dict]:
    path = out / "comparaciones.jsonl"
    if not path.exists():
        return {}
    rows = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    return {r["comparacion"]: r for r in rows}


def report(out: Path) -> int:
    done = load_done(out)
    if not done:
        print(f"[racing] sin comparaciones en {out}")
        return 1
    print(f"{'comparación':<40} {'cara a cara':>12} {'V-E-D':>8} {'vs rule_based':>14} {'rondas':>6}  veredicto")
    for r in done.values():
        print(f"{r['comparacion']:<40} {r['h2h']:>+6.2f}±{r['h2h_sem']:<5.2f} "
              f"{'-'.join(map(str, r['wdl'])):>8} {r['rb']:>+7.2f}±{r['rb_sem']:<5.2f} {r['rondas']:>6}  "
              f"{r['veredicto']}{' (veto rb)' if r['veto_rule_based'] else ''}"
              f"{'  ACEPTADO' if r['accepted'] else ''}")
    best = out / "best.json"
    if best.exists():
        print(f"\ntitular: {best}")
        print(best.read_text(encoding="utf-8"))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="búsqueda por coordenadas con racing")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--grid", action="append", default=[], help="param=v1,v2,... (repetible)")
    ap.add_argument("--start", type=Path, default=None, help="VSSL_PARAMS inicial (default: team_params.json)")
    ap.add_argument("--sweeps", type=int, default=2)
    ap.add_argument("--min-rounds", type=int, default=4)
    ap.add_argument("--max-rounds", type=int, default=12)
    ap.add_argument("--z", type=float, default=2.0, help="errores estándar para decidir")
    ap.add_argument("--seed", type=int, default=100)
    ap.add_argument("--fps", type=int, default=480)
    ap.add_argument("--minutes", type=float, default=3.0)
    ap.add_argument("--goals", type=int, default=3)
    ap.add_argument("--sims", type=int, default=1)
    ap.add_argument("--vision-port", type=int, default=10002)
    ap.add_argument("--cmd-port", type=int, default=20011)
    ap.add_argument("--referee-port", type=int, default=10003)
    ap.add_argument("--report", action="store_true")
    args = ap.parse_args()
    if args.report:
        return report(args.out)
    if not args.grid:
        sys.exit("[racing] falta al menos un --grid param=v1,v2")

    args.out.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    ranges = load_ranges()
    grid = parse_grid(args.grid, ranges)
    incumbent = load_defaults(ranges)
    if args.start is not None:
        incumbent.update({k: float(v) for k, v in
                          json.loads(args.start.read_text(encoding="utf-8-sig"))["coach"].items()})
    state_path = args.out / "best.json"
    if state_path.exists():
        incumbent.update({k: float(v) for k, v in
                          json.loads(state_path.read_text(encoding="utf-8"))["coach"].items()})
    done = load_done(args.out)

    sims = [Simulator(args.fps, args.vision_port + 10 * k, args.cmd_port + 10 * k, args.out / f"firasim_{k}.log")
            for k in range(max(1, args.sims))]
    runners: "queue.Queue[MatchRunner]" = queue.Queue()
    for k, sim in enumerate(sims):
        runners.put(MatchRunner(sim, args.minutes, args.goals, args.referee_port + k))
    pool = ThreadPoolExecutor(max_workers=len(sims))
    racer = Racer(runners, pool, args.out, args.min_rounds, args.max_rounds, args.z)

    def stop_all() -> None:
        for sim in sims:
            sim.close()

    def on_signal(_sig, _frame):
        print("[racing] interrumpido; se reanuda con la misma --out")
        stop_all(); racer.close()
        sys.exit(130)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    for sim in reversed(sims):
        sim.start()
    started = time.monotonic()
    print(f"[racing] {len(grid)} parámetros, {sum(len(v) for _, v in grid)} valores, "
          f"{args.sweeps} barridas, {len(sims)} simulador(es)", flush=True)
    comparison_index = 0
    try:
        for sweep in range(args.sweeps):
            for name, values in grid:
                for value in values:
                    comparison_index += 1
                    if abs(incumbent[name] - value) < 1e-9:
                        continue
                    label = f"b{sweep}_{name}={value:g}"
                    if label in done:
                        if done[label]["accepted"]:
                            incumbent[name] = value
                        continue
                    candidate = dict(incumbent, **{name: value})
                    print(f"[racing] {label}: titular {name}={incumbent[name]:g}", flush=True)
                    summary = racer.race(label, candidate, incumbent, args.seed + 100 * comparison_index)
                    done[label] = summary
                    if summary["accepted"]:
                        incumbent = candidate
                        write_params_file(state_path, incumbent)
                        print(f"[racing] nuevo titular: {name}={value:g}", flush=True)
        write_params_file(state_path, incumbent)
    finally:
        stop_all(); racer.close()
        pool.shutdown(wait=False)
    print(f"[racing] {time.monotonic() - started:.0f} s de pared\n")
    return report(args.out)


if __name__ == "__main__":
    sys.exit(main())
