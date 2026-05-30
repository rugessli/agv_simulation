"""Inspect pair_times for scenario B DLTO."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from simulation import run_scenario

agvs, _, makespan, pair_times, mcd, dist = run_scenario("B", "DLTO", seed=0)
print(f"makespan: {makespan:.2f} s   all_arrived: {all(a.arrived for a in agvs)}")
print(f"pair_times:")
for k, v in pair_times.items():
    latency = (v["resolved"] - v["detect"]) if v["resolved"] is not None else None
    print(f"  pair {k}: detect={v['detect']:.2f}s, resolved={v['resolved']}, latency={latency}")
print(f"\nfinal AGV states:")
for a in agvs:
    print(f"  AGV{a.aid}: x={a.x:.2f}, y={a.y:.2f}, arrived={a.arrived} at t={a.arrival_time:.2f}")
