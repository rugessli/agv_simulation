"""Re-generate all 5 scenario GIFs without re-running the full study."""
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from simulation import run_scenario
from analysis   import animate_side_by_side

OUTPUT_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output")
os.makedirs(OUTPUT_DIR, exist_ok=True)

SCENARIOS = [
    {"name": "A",     "label": "2 AGVs head-on",                "n_agv": 2},
    {"name": "B",     "label": "4 AGVs head-on",                "n_agv": 4},
    {"name": "C_TOP", "label": "Lane + obstacle on TOP wall",   "n_agv": 2},
    {"name": "C_BOT", "label": "Lane + obstacle on BOTTOM wall","n_agv": 2},
    {"name": "C_MID", "label": "Lane + obstacle on CENTRE line","n_agv": 2},
]

for scn in SCENARIOS:
    _, traj_dlto, _, _, _, _, obs_d, ev_d = run_scenario(
        scn["name"], "DLTO", seed=0, record_trajectory=True)
    _, traj_sw,   _, _, _, _, obs_s, ev_s = run_scenario(
        scn["name"], "SW",   seed=0, record_trajectory=True)
    gif_path = os.path.join(OUTPUT_DIR, f"trajectory_scenario_{scn['name']}.gif")
    animate_side_by_side(
        traj_dlto, traj_sw,
        n_agv=scn["n_agv"],
        scenario_label=scn["label"],
        save_path=gif_path,
        obstacles=obs_d,
        events_dlto=ev_d,
        events_sw=ev_s,
    )
    print(f"saved {gif_path}")

print("Done.")
