#!/usr/bin/env python3
"""Sensitivity analysis: DLTO MCD advantage robustness vs alpha (cargo-state weight).

Fixed ratio: beta=0.25, gamma=0.20, eps=0.05 (relative to alpha_base=0.50).
For each alpha, the other three weights are scaled proportionally so sum=1.

Scope: mixed payload, n_agv=2, corridors 3.2m and 3.6m, 30 seeds.
Runs: 5 alpha × 2 corridors × 30 seeds = 300 DLTO runs + 60 ORCA baseline runs.
"""

from __future__ import annotations
import os
import sys
import time
import csv

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import config
import dlto_algorithm
from simulation import run_scenario

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

ALPHA_VALUES    = [0.40, 0.45, 0.50, 0.55, 0.60]
CORRIDOR_WIDTHS = [3.2, 3.6]
N_AGV           = 2
PAYLOAD         = "mixed"
N_SEEDS         = 30

ALPHA_BASE = 0.50
BETA_BASE  = 0.25
GAMMA_BASE = 0.20
EPS_BASE   = 0.05


def _set_weights(alpha: float) -> tuple:
    scale = (1.0 - alpha) / (1.0 - ALPHA_BASE)
    beta  = round(BETA_BASE  * scale, 8)
    gamma = round(GAMMA_BASE * scale, 8)
    eps   = round(EPS_BASE   * scale, 8)
    # Patch dlto_algorithm globals (read at call time by _priority_score)
    dlto_algorithm.PRIORITY_ALPHA   = alpha
    dlto_algorithm.PRIORITY_BETA    = beta
    dlto_algorithm.PRIORITY_GAMMA   = gamma
    dlto_algorithm.PRIORITY_EPSILON = eps
    return beta, gamma, eps


def _run_batch(algorithm: str, corridor_width: float) -> list[float]:
    mcds = []
    for seed in range(N_SEEDS):
        _, _, _, _, mcd, _, _ = run_scenario(
            n_agv=N_AGV,
            algorithm=algorithm,
            seed=seed,
            corridor_width=corridor_width,
            corridor_length=config.CORRIDOR_LENGTH,
            payload_profile=PAYLOAD,
        )
        mcds.append(mcd)
    return mcds


def main() -> None:
    t0 = time.time()

    # ── ORCA baseline (independent of alpha) ──────────────────────────────────
    print("[*] Running ORCA baseline (2 corridors × 30 seeds = 60 runs)…")
    orca_mcds: dict[float, list[float]] = {}
    for w in CORRIDOR_WIDTHS:
        orca_mcds[w] = _run_batch("ORCA", w)
        mean = sum(orca_mcds[w]) / len(orca_mcds[w])
        print(f"    ORCA w={w:.1f}m: mean MCD = {mean:.4f} m")

    # ── DLTO for each alpha ────────────────────────────────────────────────────
    total_dlto = len(ALPHA_VALUES) * len(CORRIDOR_WIDTHS) * N_SEEDS
    done = 0
    rows: list[dict] = []

    for alpha in ALPHA_VALUES:
        beta, gamma, eps = _set_weights(alpha)
        print(f"\n[*] α={alpha:.2f}  β={beta:.4f}  γ={gamma:.4f}  ε={eps:.4f}")

        for w in CORRIDOR_WIDTHS:
            dlto_mcds: list[float] = []
            for seed in range(N_SEEDS):
                _, _, _, _, mcd, _, _ = run_scenario(
                    n_agv=N_AGV,
                    algorithm="DLTO",
                    seed=seed,
                    corridor_width=w,
                    corridor_length=config.CORRIDOR_LENGTH,
                    payload_profile=PAYLOAD,
                )
                dlto_mcds.append(mcd)
                done += 1
                pct    = 100.0 * done / total_dlto
                filled = int(40 * done // total_dlto)
                bar    = "#" * filled + "-" * (40 - filled)
                elapsed = time.time() - t0
                eta = (elapsed / done) * (total_dlto - done)
                print(f"\r  [{bar}] {pct:5.1f}%  ETA {eta:.0f}s  "
                      f"α={alpha} w={w} seed={seed:02d}",
                      end="", flush=True)

            dlto_mean = sum(dlto_mcds) / len(dlto_mcds)
            orca_mean = sum(orca_mcds[w]) / len(orca_mcds[w])
            improv    = 100.0 * (dlto_mean - orca_mean) / max(orca_mean, 1e-9)
            rows.append({
                "alpha":          alpha,
                "corridor_width": w,
                "DLTO_MCD_mean":  round(dlto_mean, 6),
                "ORCA_MCD_mean":  round(orca_mean, 6),
                "improvement_pct": round(improv, 4),
            })

    print()

    # ── Save CSV ───────────────────────────────────────────────────────────────
    csv_path = os.path.join(OUTPUT_DIR, "sensitivity_results.csv")
    fields = ["alpha", "corridor_width", "DLTO_MCD_mean", "ORCA_MCD_mean",
              "improvement_pct"]
    with open(csv_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)
    print(f"\n[+] Saved: {csv_path}")

    # ── Print table ────────────────────────────────────────────────────────────
    print("\n" + "=" * 72)
    print(f"  {'α':>4}  {'w(m)':>5}  {'DLTO MCD':>10}  {'ORCA MCD':>10}  "
          f"{'Improv %':>10}  Status")
    print("-" * 72)
    all_positive = True
    for r in rows:
        ok = r["improvement_pct"] > 0
        if not ok:
            all_positive = False
        status = "OK  " if ok else "FAIL"
        print(f"  {r['alpha']:.2f}  {r['corridor_width']:>5.1f}  "
              f"{r['DLTO_MCD_mean']:>10.4f}  {r['ORCA_MCD_mean']:>10.4f}  "
              f"{r['improvement_pct']:>9.2f}%  [{status}]")
    print("=" * 72)

    verdict = ("[PASS] All 5 alpha values show positive DLTO MCD improvement over ORCA."
               if all_positive else
               "[FAIL] One or more alpha values show non-positive DLTO MCD improvement.")
    print(f"\n{verdict}")
    print(f"\n[+] Total time: {time.time() - t0:.1f} s")


if __name__ == "__main__":
    main()
