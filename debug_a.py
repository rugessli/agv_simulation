"""Trace scenario A DLTO."""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from simulation import run_scenario

(agvs, traj, makespan, pair_times, mcd, dist, obs, events) = \
    run_scenario("A", "DLTO", seed=0, record_trajectory=True)
print(f"makespan: {makespan:.2f}  arrived: {[a.arrived for a in agvs]}")
print(f"pair_times: {pair_times}")
print(f"final: AGV0(x={agvs[0].x:.2f}, y={agvs[0].y:+.3f}, θ={math.degrees(agvs[0].theta):+.1f}) "
      f"AGV1(x={agvs[1].x:.2f}, y={agvs[1].y:+.3f}, θ={math.degrees(agvs[1].theta):+.1f})")
print("\ntrajectory every 0.5 s:")
for i, (t, snap) in enumerate(traj):
    if int(round(t*20)) % 10 == 0:
        a0 = snap[0]; a1 = snap[1]
        print(f"  t={t:5.2f} | a0({a0[0]:5.2f}, {a0[1]:+.3f}, {math.degrees(a0[2]):+6.1f}°) "
              f"a1({a1[0]:5.2f}, {a1[1]:+.3f}, {math.degrees(a1[2]):+6.1f}°)")
