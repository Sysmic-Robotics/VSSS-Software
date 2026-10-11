#!/usr/bin/env python3
"""Cribado de sensibilidad de los parámetros de estilo: ¿cuáles mueven el resultado?

Para cada parámetro de config/style_ranges.json se juegan dos variantes del default, con el
parámetro en el extremo bajo y en el extremo alto de su rango, cara a cara contra el default
en ambos lados de cancha y con la misma semilla de ruido por ronda. El efecto de una variante
es la diferencia de goles por partido a su favor (media ± error estándar). Es reanudable: los
partidos ya jugados quedan en partidos.jsonl y no se repiten.

    python3 tools/screening.py --rounds 6 --sims 3 --out logs/annealing/cribado
    python3 tools/screening.py --report --out logs/annealing/cribado
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
from dataclasses import dataclass
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from annealing import (  # noqa: E402
    INTEGER_PARAMS, WORK, MatchRunner, Rival, Simulator, load_defaults, load_ranges, write_params_file,
)


@dataclass
class Job:
    tag: str
    variant: str              # "<param>=lo" | "<param>=hi"
    variant_file: Path
    default_file: Path
    seed: int
    kickoff: str
    variant_is_blue: bool


def variants(ranges: dict[str, tuple[float, float]], defaults: dict[str, float], out: Path) -> dict[str, Path]:
    """Un archivo VSSL_PARAMS por extremo de cada parámetro (salta el extremo igual al default)."""
    files = {}
    (out / "variantes").mkdir(parents=True, exist_ok=True)
    for key, (lo, hi) in sorted(ranges.items()):
        for label, value in (("lo", lo), ("hi", hi)):
            if key in INTEGER_PARAMS:
                value = float(round(value))
            if abs(value - defaults[key]) < 1e-9:
                continue
            params = dict(defaults, **{key: value})
            files[f"{key}={label}"] = write_params_file(out / "variantes" / f"{key}_{label}.json", params)
    return files


def schedule(files: dict[str, Path], default_file: Path, rounds: int) -> list[Job]:
    jobs = []
    for name, path in files.items():
        for i in range(rounds):
            kickoff = "blue" if i % 2 == 0 else "yellow"
            safe = name.replace("=", "_")
            jobs.append(Job(f"{safe}_r{i}_v", name, path, default_file, i, kickoff, True))
            jobs.append(Job(f"{safe}_r{i}_d", name, path, default_file, i, kickoff, False))
    return jobs


def mean_sem(values: list[float]) -> tuple[float, float]:
    if len(values) < 2:
        return (values[0] if values else float("nan")), float("nan")
    return statistics.mean(values), statistics.stdev(values) / math.sqrt(len(values))


def report(out: Path) -> int:
    path = out / "partidos.jsonl"
    if not path.exists():
        print(f"[cribado] no hay partidos en {out}")
        return 1
    rows = [json.loads(l) for l in path.read_text(encoding="utf-8").splitlines() if l.strip()]
    by_variant: dict[str, list[float]] = {}
    for r in rows:
        # Diferencia de goles a favor de la variante.
        diff = r["goals_for"] - r["goals_against"]
        by_variant.setdefault(r["variant"], []).append(diff if r["variant_is_blue"] else -diff)
    table = []
    for name, diffs in by_variant.items():
        m, s = mean_sem(diffs)
        w = sum(1 for d in diffs if d > 0); l = sum(1 for d in diffs if d < 0)
        table.append((abs(m) / s if s and not math.isnan(s) and s > 0 else 0.0, name, m, s, len(diffs), w, l))
    table.sort(reverse=True)
    print(f"{'variante':<36} {'goles/partido':>14} {'± EE':>6} {'n':>3}  V-D   |efecto|/EE")
    for z, name, m, s, n, w, l in table:
        mark = " <-- señal" if z >= 2.0 else ""
        print(f"{name:<36} {m:>+14.2f} {s:>6.2f} {n:>3}  {w:>2}-{l:<2}  {z:>5.1f}{mark}")
    print(f"\n{len(rows)} partidos; variantes con |efecto| ≥ 2 EE: "
          f"{sum(1 for z, *_ in table if z >= 2.0)} de {len(table)}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description="cribado de sensibilidad de los parámetros de estilo")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--rounds", type=int, default=6, help="rondas por variante (2 partidos cada una)")
    ap.add_argument("--only", nargs="*", default=None, help="limitar a estos parámetros")
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

    args.out.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    ranges = load_ranges()
    if args.only:
        ranges = {k: v for k, v in ranges.items() if k in args.only}
    defaults = load_defaults(load_ranges())
    default_file = write_params_file(args.out / "default.json", defaults)
    files = variants(ranges, defaults, args.out)
    jobs = schedule(files, default_file, args.rounds)
    log_path = args.out / "partidos.jsonl"
    done = set()
    if log_path.exists():
        done = {json.loads(l)["tag"] for l in log_path.read_text(encoding="utf-8").splitlines() if l.strip()}
    pending = [j for j in jobs if j.tag not in done]
    print(f"[cribado] {len(files)} variantes, {len(jobs)} partidos ({len(done)} ya jugados)", flush=True)
    if not pending:
        return report(args.out)

    sims = [Simulator(args.fps, args.vision_port + 10 * k, args.cmd_port + 10 * k, args.out / f"firasim_{k}.log")
            for k in range(max(1, args.sims))]
    runners: "queue.Queue[MatchRunner]" = queue.Queue()
    for k, sim in enumerate(sims):
        runners.put(MatchRunner(sim, args.minutes, args.goals, args.referee_port + k))
    pool = ThreadPoolExecutor(max_workers=len(sims))
    log = open(log_path, "a", encoding="utf-8")

    def stop_all() -> None:
        for sim in sims:
            sim.close()

    def on_signal(_sig, _frame):
        print("[cribado] interrumpido; se reanuda con la misma --out")
        stop_all()
        sys.exit(130)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    for sim in reversed(sims):
        sim.start()
    started = time.monotonic()

    def play(job: Job) -> None:
        blue_file = job.variant_file if job.variant_is_blue else job.default_file
        rival = Rival("default" if job.variant_is_blue else job.variant, "heuristic",
                      job.default_file if job.variant_is_blue else job.variant_file)
        runner = runners.get()
        try:
            r = runner.play(blue_file, rival, job.seed, job.kickoff, tag=job.tag)
        finally:
            runners.put(runner)
        row = {**r.as_dict(), "tag": job.tag, "variant": job.variant, "variant_is_blue": job.variant_is_blue,
               "seed": job.seed}
        side = "azul" if job.variant_is_blue else "amarillo"
        print(f"[cribado]   {job.variant:<34} ({side:>8}): {r.goals_for}-{r.goals_against}  "
              f"({r.wall_seconds:.0f} s, frames perdidos {r.frames_lost_pct:.1f} %)", flush=True)
        log.write(json.dumps(row) + "\n"); log.flush()

    try:
        list(pool.map(play, pending))
    finally:
        stop_all()
        pool.shutdown(wait=False)
        log.close()
    print(f"[cribado] {time.monotonic() - started:.0f} s de pared\n")
    return report(args.out)


if __name__ == "__main__":
    sys.exit(main())
