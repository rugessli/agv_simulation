"""
metrics_logger.py
=================
Per-replication metric extraction and across-replication aggregation.

What it does
------------
Given the raw outputs of a single replication (`agvs`, `pair_times`,
`makespan`, `min_clearance`, `total_dist`), compute:

    DRR  – Deadlock Resolution Rate (1 if every detected conflict resolved
                                     within DEADLOCK_WINDOW seconds, else 0)
    ART  – Average Resolution Time  (mean of t_resolved − t_detect over
                                     all detected pairs in this rep)
    ST   – System Throughput        (AGVs reaching goal, scaled to
                                     a 10-minute window)
    MCD  – Minimum Clearance Distance between any two AGV edges
    NEC  – Normalised Energy Consumption — raw value here is the mean
                                     forward speed (total_dist / makespan);
                                     the SW vs DLTO ratio is computed
                                     downstream in analysis.py

ROS2 migration notes
--------------------
ROS2-READY :
  - `record_replication`, `aggregate`, `to_dataframe` — pure analytics, can
    sit in an "offline evaluator" node or in a logging tool.

ROS2-REPLACE :
  - none — this file never depends on a particular execution environment.
"""

from __future__ import annotations
import math
from typing import List, Dict

import pandas as pd

# Re-export so callers don't need to import simulation.py just for a constant
from simulation import DEADLOCK_WINDOW, MAX_SIM_TIME


# ---------------------------------------------------------------------------
# Per-replication record — ROS2-READY
# ---------------------------------------------------------------------------
def record_replication(scenario: str, algorithm: str, seed: int,
                       agvs, pair_times: Dict, makespan: float,
                       min_clearance: float, total_dist: float) -> Dict:

    # ---- DRR --------------------------------------------------------------
    if not pair_times:
        # No conflict ever detected → trivially resolved
        drr = 1
        art_values = []
    else:
        latencies = []
        unresolved = 0
        for k, v in pair_times.items():
            if v["resolved"] is None:
                unresolved += 1
                latencies.append(MAX_SIM_TIME - v["detect"])
            else:
                latencies.append(v["resolved"] - v["detect"])
        drr = 1 if all((l <= DEADLOCK_WINDOW and v["resolved"] is not None)
                       for l, (_, v) in zip(latencies, pair_times.items())) \
              else 0
        art_values = [l for l in latencies if not math.isnan(l)]

    art = float(sum(art_values) / len(art_values)) if art_values else 0.0

    # ---- ST: AGVs per 10-minute window ------------------------------------
    delivered = sum(1 for a in agvs if a.arrived)
    st = (delivered / makespan) * 600.0 if makespan > 1e-9 else 0.0

    # ---- NEC raw: mean forward speed of the *system*  ---------------------
    nec_raw = total_dist / makespan if makespan > 1e-9 else 0.0

    return {
        "scenario":  scenario,
        "algorithm": algorithm,
        "seed":      seed,
        "DRR":       drr,
        "ART":       art,
        "ST":        st,
        "MCD":       min_clearance,
        "NEC_raw":   nec_raw,
        "makespan":  makespan,
        "delivered": delivered,
    }


# ---------------------------------------------------------------------------
# Aggregation across replications — ROS2-READY
# ---------------------------------------------------------------------------
def to_dataframe(rows: List[Dict]) -> pd.DataFrame:
    return pd.DataFrame(rows)


def aggregate(df: pd.DataFrame) -> pd.DataFrame:
    """Mean ± std per (scenario, algorithm).  Adds the NEC normalised
    against the SW mean in the same scenario."""
    grouped = (df.groupby(["scenario", "algorithm"])
                 .agg(DRR_mean=("DRR", "mean"),
                      DRR_std =("DRR", "std"),
                      ART_mean=("ART", "mean"),
                      ART_std =("ART", "std"),
                      ST_mean =("ST",  "mean"),
                      ST_std  =("ST",  "std"),
                      MCD_mean=("MCD", "mean"),
                      MCD_std =("MCD", "std"),
                      NEC_raw_mean=("NEC_raw", "mean"),
                      NEC_raw_std =("NEC_raw", "std"),
                      makespan_mean=("makespan", "mean"))
                 .reset_index())

    # NEC normalised to the SW mean of the same scenario
    sw_ref = (grouped[grouped.algorithm == "SW"]
              .set_index("scenario")["NEC_raw_mean"].to_dict())
    grouped["NEC_norm"] = grouped.apply(
        lambda r: (r["NEC_raw_mean"] / sw_ref[r["scenario"]])
                  if sw_ref.get(r["scenario"], 0) > 1e-9 else float("nan"),
        axis=1,
    )
    return grouped


def print_summary(agg: pd.DataFrame) -> None:
    """Pretty-print mean ± std per metric."""
    print("\n================== summary (mean ± std) ==================\n")
    for _, r in agg.iterrows():
        print(f" scenario={r['scenario']}  algorithm={r['algorithm']}")
        print(f"   DRR  = {r['DRR_mean']:.3f} ± {r['DRR_std']:.3f}")
        print(f"   ART  = {r['ART_mean']:.3f} ± {r['ART_std']:.3f}  s")
        print(f"   ST   = {r['ST_mean']:.2f}  ± {r['ST_std']:.2f}   AGV/10-min")
        print(f"   MCD  = {r['MCD_mean']:.3f} ± {r['MCD_std']:.3f}  m")
        print(f"   NEC_raw  = {r['NEC_raw_mean']:.3f} ± {r['NEC_raw_std']:.3f}  m/s")
        print(f"   NEC_norm = {r['NEC_norm']:.3f}    (DLTO / SW)")
        print(f"   makespan = {r['makespan_mean']:.2f} s")
        print()
