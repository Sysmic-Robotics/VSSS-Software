#!/usr/bin/env python3
"""Comparación A/B de dos configuraciones de estilo con partidos acelerados en FIRASim.

En cada ronda i, con la misma semilla de ruido para las dos configuraciones:
    A vs rule_based · B vs rule_based · A (azul) vs B (amarillo) · B (azul) vs A (amarillo)

Informa, para cada configuración, aptitud media ± error estándar contra rule_based y el
cara a cara entre ambas (diferencia de goles desde el punto de vista de A, en los dos
sentidos de cancha). La palabra `default` como archivo usa config/team_params.json.

    python3 tools/ab_params.py default logs/annealing/corrida2/best.json --rounds 12 --sims 3 --out logs/annealing/ab_88
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
    WORK, MatchResult, MatchRunner, Rival, Simulator, load_defaults, load_ranges, write_params_file,
)


@dataclass
class Job:
    tag: str
    blue: str                 # "A" | "B"
    blue_file: Path
    rival: Rival              # rule_based o la otra configuración
    seed: int
    kickoff: str


def resolve_params(arg: str, label: str, out: Path) -> Path:
    if arg == "default":
        return write_params_file(out / "default.json", load_defaults(load_ranges()))
    path = Path(arg)
    if not path.exists():
        sys.exit(f"[ab] no existe {path}")
    data = json.loads(path.read_text(encoding="utf-8-sig"))
    if "coach" not in data:
        sys.exit(f"[ab] {path} no tiene la clave 'coach' (formato de VSSL_PARAMS)")
    return path


def schedule(rounds: int, a: Path, b: Path) -> list[Job]:
    jobs = []
    for i in range(rounds):
        kickoff = "blue" if i % 2 == 0 else "yellow"
        rb = Rival("rule_based", "rule_based", None)
        jobs += [
            Job(f"r{i}_A_rb", "A", a, rb, i, kickoff),
            Job(f"r{i}_B_rb", "B", b, rb, i, kickoff),
            Job(f"r{i}_A_vs_B", "A", a, Rival("B", "heuristic", b), i, kickoff),
            Job(f"r{i}_B_vs_A", "B", b, Rival("A", "heuristic", a), i, kickoff),
        ]
    return jobs


def mean_sem(values: list[float]) -> tuple[float, float]:
    if not values:
        return float("nan"), float("nan")
    if len(values) < 2:
        return values[0], float("nan")
    return statistics.mean(values), statistics.stdev(values) / math.sqrt(len(values))


def record_line(blue: str, rival: str, r: MatchResult) -> str:
    w = "V" if r.goals_for > r.goals_against else "E" if r.goals_for == r.goals_against else "D"
    return (f"{blue} vs {rival:>10}: {r.goals_for}-{r.goals_against} {w}  posesión {r.possession_pct:.0f} %  "
            f"faltas {r.area_fouls}  aptitud {r.fitness():+.2f}  ({r.wall_seconds:.0f} s, "
            f"frames perdidos {r.frames_lost_pct:.1f} %)")


def summarize(results: list[dict], names: dict[str, str]) -> str:
    lines = []
    for side in ("A", "B"):
        vs_rb = [r for r in results if r["blue"] == side and r["rival"] == "rule_based"]
        fits = [r["fitness"] for r in vs_rb]
        gd = [r["goals_for"] - r["goals_against"] for r in vs_rb]
        w = sum(1 for g in gd if g > 0); d = sum(1 for g in gd if g == 0)
        m, s = mean_sem(fits); gm, gs = mean_sem([float(g) for g in gd])
        lines.append(f"{side} = {names[side]}")
        lines.append(f"  vs rule_based ({len(vs_rb)} partidos): V{w} E{d} D{len(gd) - w - d}  "
                     f"aptitud {m:+.2f} ± {s:.2f}  dif. goles {gm:+.2f} ± {gs:.2f}  "
                     f"posesión {statistics.mean(r['possession_pct'] for r in vs_rb):.0f} %  "
                     f"faltas/partido {statistics.mean(r['area_fouls'] for r in vs_rb):.2f}")
    # Pareado por semilla contra rule_based: aptitud(B) − aptitud(A) en la misma ronda.
    by_seed_a = {r["seed"]: r["fitness"] for r in results if r["blue"] == "A" and r["rival"] == "rule_based"}
    by_seed_b = {r["seed"]: r["fitness"] for r in results if r["blue"] == "B" and r["rival"] == "rule_based"}
    paired = [by_seed_b[k] - by_seed_a[k] for k in by_seed_a if k in by_seed_b]
    pm, ps = mean_sem(paired)
    lines.append(f"Pareado vs rule_based (B − A, {len(paired)} rondas): {pm:+.2f} ± {ps:.2f} por partido")
    # Cara a cara, desde A: A azul vs B, y B azul vs A (invertido).
    h2h = []
    for r in results:
        if r["rival"] in ("A", "B"):
            diff = r["goals_for"] - r["goals_against"]
            h2h.append(diff if r["blue"] == "A" else -diff)
    w = sum(1 for g in h2h if g > 0); d = sum(1 for g in h2h if g == 0)
    hm, hs = mean_sem([float(g) for g in h2h])
    lines.append(f"Cara a cara A contra B ({len(h2h)} partidos, ambos lados): A gana {w}, empata {d}, pierde "
                 f"{len(h2h) - w - d}; dif. goles A−B {hm:+.2f} ± {hs:.2f} por partido")
    verdict = "sin diferencia detectable con estos partidos"
    if not math.isnan(pm) and abs(pm) > 2 * ps and not math.isnan(hm) and abs(hm) > 2 * hs and (pm > 0) == (hm < 0):
        verdict = f"{'B' if pm > 0 else 'A'} es mejor en ambas medidas (más de 2 errores estándar)"
    elif (not math.isnan(pm) and abs(pm) > 2 * ps) or (not math.isnan(hm) and abs(hm) > 2 * hs):
        verdict = "diferencia en una sola medida: no concluyente"
    lines.append(f"Veredicto: {verdict}")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser(description="comparación A/B de dos configuraciones de estilo")
    ap.add_argument("a", help="archivo VSSL_PARAMS de A, o 'default'")
    ap.add_argument("b", help="archivo VSSL_PARAMS de B, o 'default'")
    ap.add_argument("--rounds", type=int, default=12, help="rondas de 4 partidos")
    ap.add_argument("--out", type=Path, required=True)
    ap.add_argument("--fps", type=int, default=480)
    ap.add_argument("--minutes", type=float, default=3.0)
    ap.add_argument("--goals", type=int, default=3)
    ap.add_argument("--sims", type=int, default=1)
    ap.add_argument("--vision-port", type=int, default=10002)
    ap.add_argument("--cmd-port", type=int, default=20011)
    ap.add_argument("--referee-port", type=int, default=10003)
    ap.add_argument("--keep-work", action="store_true", help="conservar CSV y logs de cada partido en WORK")
    args = ap.parse_args()

    args.out.mkdir(parents=True, exist_ok=True)
    WORK.mkdir(parents=True, exist_ok=True)
    a = resolve_params(args.a, "A", args.out)
    b = resolve_params(args.b, "B", args.out)
    names = {"A": str(args.a), "B": str(args.b)}
    jobs = schedule(args.rounds, a, b)

    sims = [Simulator(args.fps, args.vision_port + 10 * k, args.cmd_port + 10 * k, args.out / f"firasim_{k}.log")
            for k in range(max(1, args.sims))]
    runners: "queue.Queue[MatchRunner]" = queue.Queue()
    for k, sim in enumerate(sims):
        runners.put(MatchRunner(sim, args.minutes, args.goals, args.referee_port + k, keep_work=args.keep_work))
    pool = ThreadPoolExecutor(max_workers=len(sims))
    results: list[dict] = []
    log = open(args.out / "partidos.jsonl", "a", encoding="utf-8")

    def stop_all() -> None:
        for sim in sims:
            sim.close()

    def on_signal(_sig, _frame):
        print("[ab] interrumpido")
        stop_all()
        sys.exit(130)

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    for sim in reversed(sims):
        sim.start()
    print(f"[ab] {len(jobs)} partidos en {len(sims)} simulador(es): A = {names['A']}, B = {names['B']}", flush=True)
    started = time.monotonic()

    def play(job: Job) -> dict:
        runner = runners.get()
        try:
            r = runner.play(job.blue_file, job.rival, job.seed, job.kickoff, tag=job.tag)
        finally:
            runners.put(runner)
        row = {**r.as_dict(), "tag": job.tag, "blue": job.blue, "seed": job.seed, "kickoff": job.kickoff}
        print(f"[ab]   {record_line(job.blue, job.rival.name, r)}", flush=True)
        log.write(json.dumps(row) + "\n"); log.flush()
        return row

    try:
        results = list(pool.map(play, jobs))
    finally:
        stop_all()
        pool.shutdown(wait=False)
        log.close()
    summary = summarize(results, names)
    (args.out / "resumen.txt").write_text(summary + "\n", encoding="utf-8")
    print("\n" + summary)
    print(f"[ab] {time.monotonic() - started:.0f} s de pared en total")
    return 0


if __name__ == "__main__":
    sys.exit(main())
