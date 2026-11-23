"""Regenerate output/metric_comparison.png from the obstacle experiment data.

Five metric panels (DRR / ART / ST / MCD / NEC) comparing all four algorithms
(DLTO / SW / ORCA / FLC_ORCA) across the 9 obstacle scenarios
(3 positions × 3 corridor widths).  NEC is normalised to the SW baseline of the
same scenario.  obstacle_mid @ 2.8 m is physically impassable and is annotated
as such (all algorithms dead-lock there by geometry, not by failure).
"""
from __future__ import annotations
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np
import pandas as pd

OUT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
CSV = os.path.join(OUT, "results_obstacle.csv")
PNG = os.path.join(OUT, "metric_comparison.png")

ALGOS = ["DLTO", "SW", "ORCA", "FLC_ORCA"]
ALGO_COL = {"DLTO": "#e67e22", "SW": "#7f8c8d",
            "ORCA": "#8e44ad", "FLC_ORCA": "#16a085"}

POS_ORDER   = ["obstacle_top", "obstacle_mid", "obstacle_bot"]
POS_LABEL   = {"obstacle_top": "TOP", "obstacle_mid": "MID", "obstacle_bot": "BOT"}
WIDTHS      = [2.8, 3.2, 3.6]
IMPASSABLE  = {("obstacle_mid", 2.8)}   # geometry: free side < AGV + clearances


def main() -> None:
    df = pd.read_csv(CSV)
    g = (df.groupby(["payload_profile", "corridor_width", "algorithm"])
           .agg(DRR=("DRR", "mean"), ART=("ART", "mean"), ST=("ST", "mean"),
                MCD=("MCD", "mean"), NEC_raw=("NEC_raw", "mean"))
           .reset_index())

    # scenario groups along x (position major, width minor)
    groups = [(pos, w) for pos in POS_ORDER for w in WIDTHS]
    labels = [f"{POS_LABEL[pos]}\n{w}m" for pos, w in groups]

    def cell(pos, w, algo, col):
        r = g[(g.payload_profile == pos) & (g.corridor_width == w)
              & (g.algorithm == algo)]
        return float(r[col].iloc[0]) if len(r) else np.nan

    # NEC normalised to the SW baseline of the same scenario
    def nec_norm(pos, w, algo):
        sw = cell(pos, w, "SW", "NEC_raw")
        v = cell(pos, w, algo, "NEC_raw")
        return v / sw if sw and sw > 1e-9 else np.nan

    panels = [
        ("DRR — Deadlock Resolution Rate", lambda p, w, a: cell(p, w, a, "DRR")),
        ("ART — Avg Resolution Time (s)",  lambda p, w, a: cell(p, w, a, "ART")),
        ("ST — Throughput (AGV / 10-min)", lambda p, w, a: cell(p, w, a, "ST")),
        ("MCD — Min Clearance (m)",        lambda p, w, a: cell(p, w, a, "MCD")),
        ("NEC — Normalised Energy (algo / SW)", nec_norm),
    ]

    n_groups = len(groups)
    x = np.arange(n_groups)
    bw = 0.20

    fig, axes = plt.subplots(1, 5, figsize=(30, 5.2))

    for ax, (title, fn) in zip(axes, panels):
        for k, algo in enumerate(ALGOS):
            vals = [fn(pos, w, algo) for pos, w in groups]
            ax.bar(x + (k - 1.5) * bw, vals, bw, label=algo,
                   color=ALGO_COL[algo], edgecolor="black", lw=0.4)
        ax.set_title(title, fontsize=11)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, fontsize=7)
        ax.grid(axis="y", ls=":", alpha=0.5)
        ax.margins(x=0.01)
        # mark impassable scenario columns
        for i, (pos, w) in enumerate(groups):
            if (pos, w) in IMPASSABLE:
                ax.axvspan(i - 0.5, i + 0.5, color="#cccccc", alpha=0.30, zorder=0)
    axes[0].legend(fontsize=8, loc="center left")

    fig.tight_layout()
    fig.savefig(PNG, dpi=110, bbox_inches="tight")
    plt.close(fig)
    print(f"[+] wrote {PNG}")


if __name__ == "__main__":
    main()
