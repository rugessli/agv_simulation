"""Trace C_TOP DLTO step by step."""
import sys, os, math
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from simulation import run_scenario

algo = sys.argv[1] if len(sys.argv) > 1 else "DLTO"
(agvs, traj, makespan, pair_times, mcd, dist, obs, events) = \
    run_scenario("C_TOP", algo, seed=0, record_trajectory=True)

print(f"[{algo}] makespan: {makespan:.2f}  arrived: "
      f"{[a.arrived for a in agvs]}")
print(f"pair_times: {pair_times}")
print(f"events: {events[:6]}")
print(f"final: AGV0(x={agvs[0].x:.2f}, y={agvs[0].y:+.3f}, theta={math.degrees(agvs[0].theta):+.1f}) "
      f"AGV1(x={agvs[1].x:.2f}, y={agvs[1].y:+.3f}, theta={math.degrees(agvs[1].theta):+.1f})")

print("\ntrajectory every 1 s:")
for i, (t, snap) in enumerate(traj):
    if abs(t - round(t)) < 1e-4 or i == len(traj) - 1:
        a0 = snap[0]; a1 = snap[1]
        print(f" t={t:5.2f} | a0(x={a0[0]:5.2f}, y={a0[1]:+.3f}, "
              f"θ={math.degrees(a0[2]):+6.1f}°, {a0[3]}) "
              f"a1(x={a1[0]:5.2f}, y={a1[1]:+.3f}, "
              f"θ={math.degrees(a1[2]):+6.1f}°, {a1[3]})")
