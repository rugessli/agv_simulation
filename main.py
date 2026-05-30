"""
main.py
=======
Entry point for the multi-AGV DLTO vs Stop-and-Wait study.

What it does
------------
  * Iterates over scenarios A and B × algorithms (DLTO, SW) ×
    `N_REPLICATIONS` random seeds.
  * Records per-replication metrics into a DataFrame, dumps to CSV.
  * Aggregates and prints mean ± std.
  * Renders a side-by-side GIF (DLTO on top, SW on bottom) for each
    scenario using seed=0.
  * Writes a 5-metric bar chart PNG.

ROS2 migration notes
--------------------
ROS2-REPLACE :
  - The for-loop over (scenario, algorithm, seed) becomes a CI script that
    launches the ROS2 stack repeatedly; each launch produces a bag file,
    and the analytics is run offline against the bags.
  - In a ROS2 deployment, `run_scenario` here is replaced by spinning the
    ROS executor; `analysis.py` and `metrics_logger.py` keep working
    unchanged on bag-derived DataFrames.

ROS2-READY :
  - Everything imported from `metrics_logger.py` and `analysis.py`.
"""

from __future__ import annotations
import os
import sys
import time

# Ensure same-folder imports work when run with `python main.py`
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from simulation       import run_scenario
from metrics_logger   import (record_replication, to_dataframe,
                              aggregate, print_summary)
from analysis         import (animate_side_by_side, plot_metric_bars,
                              save_csv)


# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------
N_REPLICATIONS = 80
SCENARIOS      = [
    {"name": "A",     "label": "2 AGVs head-on",                  "n_agv": 2},
    {"name": "B",     "label": "4 AGVs head-on",                  "n_agv": 4},
    {"name": "C_TOP", "label": "Lane + obstacle on TOP wall",     "n_agv": 2},
    {"name": "C_BOT", "label": "Lane + obstacle on BOTTOM wall",  "n_agv": 2},
    {"name": "C_MID", "label": "Lane + obstacle on CENTRE line",  "n_agv": 2},
]
ALGORITHMS = ["SW", "DLTO"]
OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)


# ---------------------------------------------------------------------------
def main() -> None:
    t0 = time.time()
    rows = []
    total = len(SCENARIOS) * len(ALGORITHMS) * N_REPLICATIONS
    print(f"[+] Running {total} replications ...")

    for scn in SCENARIOS:
        for algo in ALGORITHMS:
            for seed in range(N_REPLICATIONS):
                (agvs, _, makespan, pair_times, mcd, dist,
                 _obs, _events) = run_scenario(
                    name=scn["name"], algorithm=algo, seed=seed,
                )
                rows.append(record_replication(
                    scenario=scn["name"], algorithm=algo, seed=seed,
                    agvs=agvs, pair_times=pair_times,
                    makespan=makespan, min_clearance=mcd,
                    total_dist=dist,
                ))
            print(f"    scenario={scn['name']}  algo={algo:<4}  done")

    df = to_dataframe(rows)
    save_csv(df, os.path.join(OUTPUT_DIR, "results.csv"))

    agg = aggregate(df)
    save_csv(agg, os.path.join(OUTPUT_DIR, "results_summary.csv"))
    print_summary(agg)

    plot_metric_bars(agg, os.path.join(OUTPUT_DIR, "metric_comparison.png"))
    print(f"[+] saved metric_comparison.png")

    # Side-by-side GIF for each scenario (seed=0)
    for scn in SCENARIOS:
        (_, traj_dlto, _, _, _, _,
         obs_d, ev_d) = run_scenario(scn["name"], "DLTO", seed=0,
                                     record_trajectory=True)
        (_, traj_sw, _, _, _, _,
         obs_s, ev_s) = run_scenario(scn["name"], "SW",   seed=0,
                                     record_trajectory=True)
        gif_path = os.path.join(OUTPUT_DIR,
                                f"trajectory_scenario_{scn['name']}.gif")
        animate_side_by_side(traj_dlto, traj_sw,
                             n_agv=scn["n_agv"],
                             scenario_label=scn["label"],
                             save_path=gif_path,
                             obstacles=obs_d,
                             events_dlto=ev_d,
                             events_sw=ev_s)
        print(f"[+] saved {gif_path}")

    print(f"\n[+] all done in {time.time() - t0:.1f} s — output: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
