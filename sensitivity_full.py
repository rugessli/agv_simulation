#!/usr/bin/env python3
"""Sensitivity analysis reported in the paper (Section 7.5, Table 7).

alpha in {0.30, 0.40, 0.50, 0.60, 0.70}; beta, gamma, epsilon keep their
5:4:1 ratio and fill the remaining 1 - alpha.  DLTO is re-run over the full
factorial design (3 widths x 3 fleet sizes x 3 payload profiles x 30 seeds =
810 runs per alpha) and compared, seed by seed, with the ORCA runs in
output/results.csv (produced by main.py).
"""
from __future__ import annotations
import os, sys, itertools
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import pandas as pd
from scipy.stats import wilcoxon
import config, dlto_algorithm
from simulation import run_scenario
from metrics_logger import record_replication

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
ALPHAS = [0.30, 0.40, 0.50, 0.60, 0.70]


def set_alpha(a: float) -> None:
    s = (1.0 - a) / 0.5
    dlto_algorithm.PRIORITY_ALPHA = a
    dlto_algorithm.PRIORITY_BETA = 0.25 * s
    dlto_algorithm.PRIORITY_GAMMA = 0.20 * s
    dlto_algorithm.PRIORITY_EPSILON = 0.05 * s


def main() -> None:
    rows = []
    for a in ALPHAS:
        set_alpha(a)
        for w, n, p, seed in itertools.product(config.CORRIDOR_WIDTHS, config.AGV_COUNTS,
                                               config.PAYLOAD_PROFILES, range(config.N_REPLICATIONS)):
            agvs, _, mk, pt, mcd, dist, _ = run_scenario(n_agv=n, algorithm="DLTO", seed=seed,
                                                        corridor_width=w, payload_profile=p)
            r = record_replication(w, n, p, "DLTO", seed, agvs, pt, mk, mcd, dist)
            r["alpha"] = a
            rows.append(r)
        print(f"alpha={a:.2f} done", flush=True)
    sens = pd.DataFrame(rows)
    sens.to_csv(os.path.join(OUT, "sensitivity_full.csv"), index=False)

    key = ["corridor_width", "n_agv", "payload_profile", "seed"]
    base = pd.read_csv(os.path.join(OUT, "results.csv"))
    orca = base[base.algorithm == "ORCA"].set_index(key).MCD
    print("alpha | 3.2 m mixed | 3.6 m mixed | mixed, mean of widths")
    for a, g in sens.groupby("alpha"):
        g = g.set_index(key).MCD
        cells = []
        for sel in (lambda i: (i.get_level_values(0) == 3.2) & (i.get_level_values(2) == "mixed"),
                    lambda i: (i.get_level_values(0) == 3.6) & (i.get_level_values(2) == "mixed"),
                    lambda i: i.get_level_values(2) == "mixed"):
            x = g[sel(g.index)]; y = orca.loc[x.index]
            cells.append(f"{100 * (x.mean() / y.mean() - 1):+.1f}% (p={wilcoxon(x - y).pvalue:.0e})")
        print(f"{a:.2f} | " + " | ".join(cells))


if __name__ == "__main__":
    main()
