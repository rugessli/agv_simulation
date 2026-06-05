from __future__ import annotations
import math
import os

import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# CSV writer
# ---------------------------------------------------------------------------
def save_csv(df: pd.DataFrame, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    df.to_csv(path, index=False)


# ---------------------------------------------------------------------------
# Statistical tests
# ---------------------------------------------------------------------------
def cohens_d(a: np.ndarray, b: np.ndarray) -> float:
    n_a, n_b = len(a), len(b)
    if n_a < 2 or n_b < 2:
        return float("nan")
    pooled_std = math.sqrt(
        ((n_a - 1) * np.var(a, ddof=1) + (n_b - 1) * np.var(b, ddof=1))
        / (n_a + n_b - 2)
    )
    if pooled_std < 1e-12:
        return 0.0
    return (np.mean(a) - np.mean(b)) / pooled_std


def wilcoxon_p(a: np.ndarray, b: np.ndarray) -> float:
    """Wilcoxon signed-rank p-value (two-sided). Returns nan if not computable."""
    try:
        from scipy.stats import wilcoxon
        diff = a - b
        if np.all(diff == 0):
            return 1.0
        _, p = wilcoxon(diff)
        return float(p)
    except Exception:
        return float("nan")


def statistical_comparison(df: pd.DataFrame,
                            metric: str = "ART",
                            baseline: str = "SW") -> pd.DataFrame:
    """For each (corridor_width, n_agv, payload_profile, algorithm) group
    compare against baseline using Wilcoxon and Cohen's d."""
    group_cols = ["corridor_width", "n_agv", "payload_profile"]
    records = []
    for keys, sub in df.groupby(group_cols):
        cw, na, pp = keys
        base_data = sub[sub.algorithm == baseline][metric].values
        if len(base_data) == 0:
            continue
        for algo, grp in sub.groupby("algorithm"):
            if algo == baseline:
                continue
            algo_data = grp[metric].values
            n = min(len(base_data), len(algo_data))
            records.append({
                "corridor_width":  cw,
                "n_agv":           na,
                "payload_profile": pp,
                "algorithm":       algo,
                "metric":          metric,
                "baseline":        baseline,
                "mean_baseline":   float(np.mean(base_data)),
                "mean_algo":       float(np.mean(algo_data)),
                "cohens_d":        cohens_d(algo_data[:n], base_data[:n]),
                "wilcoxon_p":      wilcoxon_p(algo_data[:n], base_data[:n]),
            })
    return pd.DataFrame(records)


# ---------------------------------------------------------------------------
# Bar chart (works without animation dependency)
# ---------------------------------------------------------------------------
def plot_metric_bars(agg: pd.DataFrame, save_path: str) -> None:
    try:
        import matplotlib.pyplot as plt
    except ImportError:
        print("[!] matplotlib not available — skipping plot")
        return

    metrics = [
        ("DRR_mean", "DRR"),
        ("ART_mean", "ART (s)"),
        ("ST_mean",  "ST (AGV/10-min)"),
        ("MCD_mean", "MCD (m)"),
        ("NEC_raw_mean", "NEC_raw (m/s)"),
    ]
    algos = sorted(agg["algorithm"].unique())
    groups = agg.groupby(["corridor_width", "n_agv", "payload_profile"])
    n_groups = len(list(groups))

    fig, axes = plt.subplots(1, len(metrics), figsize=(5 * len(metrics), 4))
    if len(metrics) == 1:
        axes = [axes]

    colors = plt.cm.tab10(np.linspace(0, 1, max(len(algos), 3)))

    for ax, (col, label) in zip(axes, metrics):
        x = np.arange(n_groups)
        width = 0.8 / len(algos)
        for k, algo in enumerate(algos):
            vals = []
            for _, sub in groups:
                row = sub[sub.algorithm == algo]
                vals.append(float(row[col].values[0]) if len(row) else float("nan"))
            ax.bar(x + k * width, vals, width, label=algo, color=colors[k])
        ax.set_title(label)
        ax.set_xticks(x + width * (len(algos) - 1) / 2)
        ax.set_xticklabels([str(i) for i in range(n_groups)], fontsize=7)
        ax.legend(fontsize=7)
        ax.grid(axis="y", alpha=0.3)

    fig.tight_layout()
    fig.savefig(save_path, dpi=120)
    plt.close(fig)
    print(f"[+] saved {save_path}")
