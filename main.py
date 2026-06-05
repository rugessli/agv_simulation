from __future__ import annotations
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from config import (
    CORRIDOR_WIDTHS, AGV_COUNTS, PAYLOAD_PROFILES, ALGORITHMS, N_REPLICATIONS,
    CORRIDOR_LENGTH,
)
from simulation    import run_scenario
from metrics_logger import record_replication, to_dataframe, aggregate, print_summary

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

CSV_PATH = os.path.join(OUTPUT_DIR, "results.csv")


def _save_csv(rows: list, path: str) -> None:
    import pandas as pd
    df = to_dataframe(rows)
    df.to_csv(path, index=False)


def main() -> None:
    t0 = time.time()
    total = (len(CORRIDOR_WIDTHS) * len(AGV_COUNTS) *
             len(PAYLOAD_PROFILES) * len(ALGORITHMS) * N_REPLICATIONS)
    print(f"[+] Full factorial: {total} runs  "
          f"({len(CORRIDOR_WIDTHS)}W × {len(AGV_COUNTS)}N × "
          f"{len(PAYLOAD_PROFILES)}P × {len(ALGORITHMS)}A × {N_REPLICATIONS}R)")

    rows = []
    done = 0
    last_save = 0

    for corridor_width in CORRIDOR_WIDTHS:
        for n_agv in AGV_COUNTS:
            for payload_profile in PAYLOAD_PROFILES:
                for algorithm in ALGORITHMS:
                    for seed in range(N_REPLICATIONS):
                        (agvs, _, makespan, pair_times, mcd, dist, _) = run_scenario(
                            n_agv=n_agv,
                            algorithm=algorithm,
                            seed=seed,
                            corridor_width=corridor_width,
                            corridor_length=CORRIDOR_LENGTH,
                            payload_profile=payload_profile,
                        )
                        rows.append(record_replication(
                            corridor_width=corridor_width,
                            n_agv=n_agv,
                            payload_profile=payload_profile,
                            algorithm=algorithm,
                            seed=seed,
                            agvs=agvs,
                            pair_times=pair_times,
                            makespan=makespan,
                            min_clearance=mcd,
                            total_dist=dist,
                            has_obstacle=False,
                        ))
                        done += 1

                        if done - last_save >= 100:
                            _save_csv(rows, CSV_PATH)
                            last_save = done

                        pct = 100.0 * done / total
                        bar_w = 40
                        filled = int(bar_w * done // total)
                        bar = "#" * filled + "-" * (bar_w - filled)
                        elapsed = time.time() - t0
                        eta = (elapsed / done) * (total - done) if done else 0
                        print(f"\r[{bar}] {pct:5.1f}%  {done}/{total}  "
                              f"ETA {eta:.0f}s  "
                              f"w={corridor_width} n={n_agv} "
                              f"p={payload_profile} a={algorithm}",
                              end="", flush=True)

    print()
    _save_csv(rows, CSV_PATH)
    print(f"[+] results.csv saved ({len(rows)} rows)")

    import pandas as pd
    df = to_dataframe(rows)
    agg = aggregate(df)
    agg.to_csv(os.path.join(OUTPUT_DIR, "results_summary.csv"), index=False)
    print_summary(agg)

    print(f"\n[+] Done in {time.time() - t0:.1f} s — output: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
