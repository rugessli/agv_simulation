"""1,080 obstacle-scenario runs:
3 widths × 3 obstacle positions × 4 algorithms × 30 seeds, n_agv=2.
"""
from __future__ import annotations
import os, sys, time
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import CORRIDOR_WIDTHS, ALGORITHMS, OBSTACLE_PROFILES, OBSTACLE_SEEDS
from simulation import run_scenario, make_obstacle_set
from metrics_logger import record_replication, to_dataframe

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)
CSV_PATH = os.path.join(OUTPUT_DIR, "results_obstacle.csv")


def main():
    t0    = time.time()
    total = len(CORRIDOR_WIDTHS) * len(OBSTACLE_PROFILES) * len(ALGORITHMS) * OBSTACLE_SEEDS
    print(f"[+] Obstacle experiment: {total} runs  "
          f"({len(CORRIDOR_WIDTHS)}W × {len(OBSTACLE_PROFILES)}P × "
          f"{len(ALGORITHMS)}A × {OBSTACLE_SEEDS}S)")

    rows = []
    done = 0
    for corridor_width in CORRIDOR_WIDTHS:
        for obs_profile in OBSTACLE_PROFILES:
            obstacles = make_obstacle_set(obs_profile, corridor_width)
            for algorithm in ALGORITHMS:
                for seed in range(OBSTACLE_SEEDS):
                    agvs, _, makespan, pair_times, mcd, dist, _ = run_scenario(
                        n_agv=2, algorithm=algorithm, seed=seed,
                        corridor_width=corridor_width,
                        payload_profile=obs_profile,
                        obstacles=obstacles,
                    )
                    rows.append(record_replication(
                        corridor_width=corridor_width, n_agv=2,
                        payload_profile=obs_profile, algorithm=algorithm,
                        seed=seed, agvs=agvs, pair_times=pair_times,
                        makespan=makespan, min_clearance=mcd, total_dist=dist,
                        has_obstacle=True,
                    ))
                    done += 1
                    pct = 100.0 * done / total
                    elapsed = time.time() - t0
                    eta = (elapsed / done) * (total - done) if done else 0
                    print(f"\r  {pct:5.1f}%  {done}/{total}  ETA {eta:.0f}s  "
                          f"w={corridor_width} p={obs_profile} a={algorithm} s={seed}",
                          end="", flush=True)

    print()
    df = to_dataframe(rows)
    df.to_csv(CSV_PATH, index=False)
    print(f"[+] {len(rows)} rows -> {CSV_PATH}  ({time.time()-t0:.0f}s)")

if __name__ == "__main__":
    main()
