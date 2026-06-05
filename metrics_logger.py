from __future__ import annotations
import math
from typing import List, Dict

import pandas as pd

from config import T_MAX, T_DEAD

MAX_SIM_TIME    = T_MAX
DEADLOCK_WINDOW = T_DEAD


def record_replication(corridor_width: float, n_agv: int,
                       payload_profile: str, algorithm: str, seed: int,
                       agvs, pair_times: Dict, makespan: float,
                       min_clearance: float, total_dist: float,
                       has_obstacle: bool = False) -> Dict:

    if not pair_times:
        # No head-on encounters detected — resolution rate is 0 (nothing to resolve)
        drr = 0.0
        art_values = []
    else:
        latencies = []
        for k, v in pair_times.items():
            if v["resolved"] is None:
                latencies.append(MAX_SIM_TIME - v["detect"])
            else:
                latencies.append(v["resolved"] - v["detect"])
        encounter_count = len(pair_times)
        resolved_count = sum(1 for v in pair_times.values() if v["resolved"] is not None)
        drr = resolved_count / encounter_count
        art_values = [l for l in latencies if not math.isnan(l)]

    art = float(sum(art_values) / len(art_values)) if art_values else 0.0
    delivered = sum(1 for a in agvs if a.arrived)
    st = (delivered / makespan) * 600.0 if makespan > 1e-9 else 0.0
    nec_raw = total_dist / makespan if makespan > 1e-9 else 0.0

    return {
        "corridor_width":  corridor_width,
        "n_agv":           n_agv,
        "payload_profile": payload_profile,
        "algorithm":       algorithm,
        "seed":            seed,
        "DRR":             drr,
        "ART":             art,
        "ST":              st,
        "MCD":             min_clearance,
        "NEC_raw":         nec_raw,
        "makespan":        makespan,
        "delivered":       delivered,
        "obstacle":        has_obstacle,
    }


def to_dataframe(rows: List[Dict]) -> pd.DataFrame:
    return pd.DataFrame(rows)


def aggregate(df: pd.DataFrame) -> pd.DataFrame:
    group_cols = ["corridor_width", "n_agv", "payload_profile", "algorithm"]
    grouped = (df.groupby(group_cols)
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

    sw_ref = (grouped[grouped.algorithm == "SW"]
              .set_index(["corridor_width", "n_agv", "payload_profile"])
              ["NEC_raw_mean"].to_dict())
    grouped["NEC_norm"] = grouped.apply(
        lambda r: (r["NEC_raw_mean"] / sw_ref.get(
            (r["corridor_width"], r["n_agv"], r["payload_profile"]), float("nan")))
        if sw_ref.get((r["corridor_width"], r["n_agv"], r["payload_profile"]), 0) > 1e-9
        else float("nan"),
        axis=1,
    )
    return grouped


def print_summary(agg: pd.DataFrame) -> None:
    print("\n================== summary (mean ± std) ==================\n")
    for _, r in agg.iterrows():
        print(f" w={r['corridor_width']}  n={r['n_agv']}  "
              f"profile={r['payload_profile']}  algo={r['algorithm']}")
        print(f"   DRR={r['DRR_mean']:.3f}±{r['DRR_std']:.3f}  "
              f"ART={r['ART_mean']:.2f}s  ST={r['ST_mean']:.2f}  "
              f"MCD={r['MCD_mean']:.3f}m  NEC_raw={r['NEC_raw_mean']:.3f}")
