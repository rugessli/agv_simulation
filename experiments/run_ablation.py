import itertools, time, os
from multiprocessing import Pool
import pandas as pd
from common import *
from simulation import run_scenario
from metrics_logger import record_replication
VARIANTS = {  # name: (alpha, beta, gamma, epsilon)
  "DLTO":            (0.50, 0.25, 0.20, 0.05),
  "noCargoState":    (0.00, 0.25, 0.20, 0.05),   # alpha = 0
  "noCargoInfo":     (0.00, 0.00, 0.20, 0.05),   # alpha = beta = 0 (proximity + ID only)
}
def task(t):
    v, w, n, p, seed = t
    set_weights(*VARIANTS[v])
    agvs, traj, mk, pt, mcd, dist, _ = run_scenario(n_agv=n, algorithm='DLTO', seed=seed, corridor_width=w,
        corridor_length=config.CORRIDOR_LENGTH, payload_profile=p, record_trajectory=True)
    r = record_replication(w, n, p, 'DLTO', seed, agvs, pt, mk, mcd, dist); r['variant'] = v
    r.update(yield_stats(traj, agvs, w)); return r
def orca_task(t):
    w, n, p, seed = t
    agvs, traj, mk, pt, mcd, dist, _ = run_scenario(n_agv=n, algorithm='ORCA', seed=seed, corridor_width=w,
        corridor_length=config.CORRIDOR_LENGTH, payload_profile=p, record_trajectory=True)
    r = record_replication(w, n, p, 'ORCA', seed, agvs, pt, mk, mcd, dist); r['variant'] = 'ORCA'
    r.update(yield_stats(traj, agvs, w)); return r
if __name__ == '__main__':
    t0 = time.time()
    grid = list(itertools.product(config.CORRIDOR_WIDTHS, config.AGV_COUNTS, config.PAYLOAD_PROFILES, range(30)))
    with Pool(2) as pool:
        rows = pool.map(task, [(v,)+g for v in VARIANTS for g in grid], chunksize=20)
        rows += pool.map(orca_task, grid, chunksize=20)
    os.makedirs('results', exist_ok=True); pd.DataFrame(rows).to_csv('results/ablation.csv', index=False)
    print('done', len(rows), round(time.time()-t0), 's')
