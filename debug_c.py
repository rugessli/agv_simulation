"""Trace Scenario C DLTO step by step."""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from simulation import run_scenario, DT

(agvs, traj, makespan, pair_times, mcd, dist, obs, events) = \
    run_scenario("C", "DLTO", seed=0, record_trajectory=True)

print(f"makespan: {makespan:.2f}  pair_times: {pair_times}")
print(f"events: {events[:5]} ... ({len(events)} total)")
print(f"final positions:")
for a in agvs:
    print(f"  AGV{a.aid}: x={a.x:.2f}, y={a.y:.2f}, theta={math.degrees(a.theta):.1f}°, arrived={a.arrived}")

print(f"\ntrajectory samples (every 1s):")
for i, (t, snap) in enumerate(traj):
    if i % 20 == 0:
        ag0 = snap[0]; ag1 = snap[1]
        print(f"  t={t:5.2f} | AGV0(x={ag0[0]:5.2f}, y={ag0[1]:+.3f}, "
              f"θ={math.degrees(ag0[2]):+6.1f}°) "
              f"AGV1(x={ag1[0]:5.2f}, y={ag1[1]:+.3f}, "
              f"θ={math.degrees(ag1[2]):+6.1f}°)")
