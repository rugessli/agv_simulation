"""Max-offset reference baseline and wall clearance of loaded vehicles (paper Section 6.10).

Runs DLTO, ORCA, FLC-ORCA and MAX_OFFSET (both vehicles take the maximum
admissible offset) over the full factorial design and records, per run, the
inter-vehicle MCD and the minimum distance between the footprint of any
loaded vehicle and the corridor walls during the run.
"""
import itertools, math, os, time
from multiprocessing import Pool
from types import SimpleNamespace
import pandas as pd
from common import *
from simulation import run_scenario
from metrics_logger import record_replication
from agv_model import corners

ALGS = ["DLTO", "ORCA", "FLC_ORCA", "MAX_OFFSET"]

def wall_stats(traj, agvs, w):
    hw = w / 2.0; lw = {True: math.inf, False: math.inf}; my = [0.0] * len(agvs)
    for _, st in traj:
        for k, s in enumerate(st):
            if s[4]: continue
            c = corners(SimpleNamespace(x=s[0], y=s[1], theta=s[2]))
            clr = hw - max(abs(p[1]) for p in c)
            lw[bool(agvs[k].cargo_state)] = min(lw[bool(agvs[k].cargo_state)], clr)
            my[k] = max(my[k], abs(s[1]))
    out = {"wall_loaded": lw[True] if lw[True] < math.inf else None,
           "wall_unloaded": lw[False] if lw[False] < math.inf else None}
    L = [my[k] for k, a in enumerate(agvs) if a.cargo_state]
    if L: out["maxy_loaded"] = sum(L) / len(L)
    return out

def task(t):
    alg, w, n, p, seed = t
    agvs, traj, mk, pt, mcd, dist, _ = run_scenario(n_agv=n, algorithm=alg, seed=seed, corridor_width=w,
        corridor_length=config.CORRIDOR_LENGTH, payload_profile=p, record_trajectory=True)
    r = record_replication(w, n, p, alg, seed, agvs, pt, mk, mcd, dist); r.update(wall_stats(traj, agvs, w)); return r

if __name__ == "__main__":
    t0 = time.time()
    T = [(a,) + g for a in ALGS for g in itertools.product(config.CORRIDOR_WIDTHS, config.AGV_COUNTS,
                                                           config.PAYLOAD_PROFILES, range(30))]
    with Pool(2) as pool:
        rows = pool.map(task, T, chunksize=20)
    os.makedirs("results", exist_ok=True)
    pd.DataFrame(rows).to_csv("results/max_offset.csv", index=False)
    print("done", len(rows), round(time.time() - t0), "s")
