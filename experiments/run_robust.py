"""Robustness to pose-report period, latency, packet loss and localisation noise.

Each AGV plans with (i) its own pose estimate = true pose + localisation error, and
(ii) the latest pose reports it has received from the other vehicles. Reports are
broadcast periodically, arrive after a latency and are lost independently per receiver.
The last report of each sender is propagated at its reported velocity (dead reckoning);
with DISCARD_AFTER_TIMEOUT=1 a sender is instead discarded after REPORT_TIMEOUT. Within the laser range (DETECT_RANGE) the other vehicle's pose is measured
directly (relative to the receiver, so only the receiver's own localisation error applies),
while its intended offset is still known only from its last report. Algorithms, controller and safety layer are unchanged.
"""
import itertools, time, math, copy, sys, os
from multiprocessing import Pool
import numpy as np, pandas as pd
from common import *
import simulation as S
from simulation import make_scenario, step_dlto, safe_motion, _apply_following_distance, \
    _sync_orca_state, pairwise_min_obb_distance, passed
from orca_algorithm import orca_step
from flc_orca_algorithm import flc_orca_step
from agv_model import integrate, steering_omega, has_arrived, corners
from metrics_logger import record_replication
from config import *

REPORT_TIMEOUT = 2.0
EXTRAPOLATE = os.environ.get("DISCARD_AFTER_TIMEOUT") != "1"   # DISCARD_AFTER_TIMEOUT=1: variant of Section 6.9
TAU_LOC = 2.0          # correlation time of the localisation error (s)
CONDITIONS = {         # name: (period s, latency s, loss prob, loc sigma m)
  "C0_ideal":        (0.1, 0.0, 0.0, 0.00),
  "C1_0.5s":         (0.5, 0.1, 0.0, 0.00),
  "C2_1.0s":         (1.0, 0.2, 0.0, 0.00),
  "C3_loss30":       (0.5, 0.1, 0.3, 0.00),
  "C4_loc2cm":       (0.5, 0.1, 0.0, 0.02),
  "C5_loc5cm":       (0.5, 0.1, 0.0, 0.05),
  "C6_worst":        (1.0, 0.3, 0.3, 0.05),
}

def run(n_agv, algorithm, seed, w, profile, cond):
    period, lat, loss, sig = CONDITIONS[cond]
    agvs = make_scenario(n_agv, seed, w, CORRIDOR_LENGTH, profile)
    rng = np.random.default_rng(10_000 + seed * 97 + list(CONDITIONS).index(cond))
    hw = w / 2.0; pair_times = {}; t = 0.0; mcd = math.inf; wall = math.inf
    rho = math.exp(-DT / TAU_LOC)
    err = {a.aid: rng.normal(0, sig, 2) for a in agvs}
    phase = {a.aid: rng.uniform(0, period) for a in agvs}
    last_k = {a.aid: -1 for a in agvs}
    inbox = {a.aid: {} for a in agvs}       # receiver -> sender -> (t_meas, snapshot)
    pending = []                            # (t_arrive, receiver, sender, t_meas, snapshot)
    def est(a):
        e = copy.copy(a); e.x = a.x + err[a.aid][0]; e.y = a.y + err[a.aid][1]; return e
    while t < T_MAX:
        # --- broadcasts ---
        for s in agvs:
            k = int(math.floor((t - phase[s.aid]) / period)) if t >= phase[s.aid] else -1
            if k > last_k[s.aid]:
                last_k[s.aid] = k; snap = est(s)
                for r in agvs:
                    if r is s or rng.random() < loss: continue
                    pending.append((t + lat, r.aid, s.aid, t, snap))
        keep = []
        for item in pending:
            if item[0] <= t + 1e-9:
                _, r, s_, tm, snap = item
                if s_ not in inbox[r] or inbox[r][s_][0] < tm: inbox[r][s_] = (tm, snap)
            else: keep.append(item)
        pending = keep
        # --- planning: each vehicle on its own view ---
        for a in agvs:
            if a.arrived: continue
            me = est(a); view = [me]
            for b in agvs:
                if b is a: continue
                rec = inbox[a.aid].get(b.aid)
                fresh = rec is not None and t - rec[0] <= REPORT_TIMEOUT
                if not b.arrived and math.hypot(a.x - b.x, a.y - b.y) <= DETECT_RANGE:
                    # within laser range: relative pose measured directly (accurate relative to
                    # the receiver, so it inherits only the receiver's own localisation error);
                    # the partner's intended offset is known only from its last report
                    p = copy.copy(b); p.x = b.x + err[a.aid][0]; p.y = b.y + err[a.aid][1]
                    p.target_y = rec[1].target_y if fresh else p.y
                elif rec is not None and (fresh or EXTRAPOLATE):
                    # last report, propagated at its reported velocity (dead reckoning)
                    p = copy.copy(rec[1]); age = t - rec[0]
                    if not p.arrived:
                        p.x += p.v * math.cos(p.theta) * age; p.y += p.v * math.sin(p.theta) * age
                else:
                    p = copy.copy(b); p.arrived = True       # never heard of -> ignored by planner
                view.append(p)
            if algorithm == "DLTO": step_dlto(view, t, {}, w, None)
            elif algorithm == "ORCA": orca_step(view, hw)
            else: flc_orca_step(view, hw)
            a.target_y, a.state = me.target_y, me.state
        _sync_orca_state(agvs, t, pair_times)                 # ground-truth encounter log
        by = {a.aid: a for a in agvs}
        for key, pt in pair_times.items():
            if pt["resolved"] is None:
                x1, x2 = by[key[0]], by[key[1]]
                if x1.arrived or x2.arrived or passed(x1, x2) or passed(x2, x1): pt["resolved"] = t
        _apply_following_distance(agvs)                        # laser-based, true geometry
        for a in agvs:
            if a.arrived: a.v = 0.0; continue
            if t < a.departure_delay: integrate(a, 0.0, 0.0, DT, hw); continue
            v = {"WAIT": 0.0, "CREEP": CREEP_SPEED, "SLOW": OFFSET_SPEED * a.speed_factor,
                 "APPROACH": APPROACH_SPEED * a.speed_factor}.get(a.state, CRUISE_SPEED * a.speed_factor)
            stp = OFFSET_RATE * DT
            a.smooth_target_y += max(-stp, min(stp, a.target_y - a.smooth_target_y))
            om = 0.0 if a.state == "WAIT" else steering_omega(est(a), v=v, corridor_half_w=hw, y_target=a.smooth_target_y, dt=DT)
            safe_motion(a, v, om, DT, agvs, hw, None)
            if has_arrived(a): a.arrived = True; a.arrival_time = t
        for a in agvs:                                          # localisation error update
            err[a.aid] = rho * err[a.aid] + math.sqrt(1 - rho * rho) * rng.normal(0, sig, 2)
        live = [a for a in agvs if not a.arrived]
        for i in range(len(live)):
            ci = corners(live[i])
            wall = min(wall, hw - max(abs(c[1]) for c in ci))
            for j in range(i + 1, len(live)):
                if live[i].direction != live[j].direction:
                    mcd = min(mcd, pairwise_min_obb_distance(ci, corners(live[j])))
        if all(a.arrived for a in agvs): break
        t += DT
    mk = max((a.arrival_time for a in agvs if a.arrived), default=t)
    dist = sum(a.distance_travelled for a in agvs)
    if math.isinf(mcd): mcd = hw
    r = record_replication(w, n_agv, profile, algorithm, seed, agvs, pair_times, mk, mcd, dist)
    r["condition"] = cond; r["wall_clearance"] = wall; return r

def task(t): return run(t[1], t[0], t[4], t[2], t[3], t[5])

if __name__ == "__main__":
    t0 = time.time()
    conds = sys.argv[1].split(",") if len(sys.argv) > 1 else list(CONDITIONS)
    seeds = int(sys.argv[2]) if len(sys.argv) > 2 else 30
    T = [(alg, n, w, p, s, c) for c in conds for alg in ["DLTO", "ORCA", "FLC_ORCA"]
         for w in CORRIDOR_WIDTHS for n in AGV_COUNTS for p in PAYLOAD_PROFILES for s in range(seeds)]
    with Pool(2) as pool:
        rows = pool.map(task, T, chunksize=10)
    out = sys.argv[3] if len(sys.argv) > 3 else "results/robustness.csv"
    pd.DataFrame(rows).to_csv(out, index=False); print("done", len(rows), round(time.time() - t0), "s")
