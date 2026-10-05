"""Generate all comparison GIFs for main and obstacle experiments.

Main experiment  : 3 corridors × 3 payloads  = 9 comparison GIFs
Obstacle experiment: 3 corridors × 3 positions = 9 comparison GIFs

Each GIF: 4×1 layout (DLTO / SW / ORCA / FLC_ORCA), n_agv=2, seed=0.
figsize=(32, 16), dpi=80, ylim dynamic per corridor width.
Naming:
  main     → w{corridor}_n2_{payload}_s0.gif
  obstacle → w{corridor}_n2_{obstacle_type}_s0.gif
"""
from __future__ import annotations
import glob
import math
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import matplotlib
matplotlib.use("Agg")
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.transforms import Affine2D

from config import (
    AGV_LENGTH, AGV_WIDTH, CORRIDOR_LENGTH, DT,
    CORRIDOR_WIDTHS, PAYLOAD_PROFILES, OBSTACLE_PROFILES,
    ALGORITHMS,
)
from simulation import run_scenario, pairwise_min_obb_distance, make_obstacle_set

GIF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "gifs")
os.makedirs(GIF_DIR, exist_ok=True)

N_AGV      = 2
SEED       = 0
FPS        = 10
MAX_FRAMES = 180
DPI        = 80

ALGO_ORDER = ["DLTO", "SW", "ORCA", "FLC_ORCA"]

# For "mixed" and obstacle profiles the cargo of each AGV is drawn at random;
# seed 0 happens to leave BOTH vehicles unloaded (both blue).  Seed 1 yields one
# loaded + one empty, so the red/blue cargo distinction is clearly visible.
# "unloaded"/"loaded" force the cargo state, so their seed does not matter.
SPLIT_CARGO_SEED = 1


def seed_for(profile: str) -> int:
    return SPLIT_CARGO_SEED if profile in (
        "mixed", "obstacle_top", "obstacle_mid", "obstacle_bot") else SEED

_ALGO_COL = {
    "DLTO":     "#e67e22",
    "SW":       "#c0392b",
    "ORCA":     "#8e44ad",
    "FLC_ORCA": "#16a085",
}
_COL_LOADED   = "#e74c3c"   # cargo-laden AGV → red
_COL_UNLOADED = "#3498db"   # empty AGV       → blue
_COL_ARRIVED  = "#bbbbbb"
_COL_DETOUR   = "#e74c3c"
_COL_NORMAL   = "#2c3e50"


# ── geometry ──────────────────────────────────────────────────────────────────
def _obb_corners(x, y, theta):
    c, s = math.cos(theta), math.sin(theta)
    l2, w2 = AGV_LENGTH / 2, AGV_WIDTH / 2
    return [(x + lx * c - ly * s, y + lx * s + ly * c)
            for lx, ly in [(l2, w2), (l2, -w2), (-l2, -w2), (-l2, w2)]]


def _live_mcd(snap):
    live = [(i, e) for i, e in enumerate(snap) if not e[4]]
    best = math.inf
    for i, ei in live:
        for j, ej in live:
            if j <= i:
                continue
            dth = abs((ei[2] - ej[2] + math.pi) % (2 * math.pi) - math.pi)
            if dth < math.radians(90):
                continue
            d = pairwise_min_obb_distance(
                _obb_corners(ei[0], ei[1], ei[2]),
                _obb_corners(ej[0], ej[1], ej[2]))
            if d < best:
                best = d
    return best if not math.isinf(best) else 0.0


def _sim_status(snap, pair_times, t):
    if all(e[4] for e in snap):
        return "COMPLETED"
    if any(v["resolved"] is None and (t - v["detect"]) > 25.0
           for v in pair_times.values()):
        return "DEADLOCKED"
    if any(e[3] in ("SLOW", "WAIT") for e in snap if not e[4]):
        return "OFFSETTING"
    return "RUNNING"


def _snap_at(traj, t_req):
    for ft, snap in traj:
        if ft >= t_req:
            return ft, snap
    return traj[-1]


# ── subplot helpers ────────────────────────────────────────────────────────────
def _init_subplot(ax, algorithm, corridor_width, n_agv, obstacles=None):
    half_w = corridor_width / 2.0
    y_pad  = 0.2

    ax.set_xlim(-0.5, CORRIDOR_LENGTH + 0.5)
    ax.set_ylim(-(half_w + y_pad), half_w + y_pad)
    ax.set_aspect("equal")
    ax.set_facecolor("#f5f5f5")
    ax.tick_params(labelsize=7)
    ax.set_xlabel("x (m)", fontsize=9)

    ax.add_patch(mpatches.Rectangle(
        (0, -half_w), CORRIDOR_LENGTH, 2 * half_w,
        fill=False, lw=1.4, edgecolor="black", zorder=3))
    ax.axhline(0, ls="--", lw=0.5, color="#aaaaaa", alpha=0.5, zorder=1)

    col = _ALGO_COL[algorithm]
    ax.text(0.01, 0.96, f"  {algorithm}  ",
            transform=ax.transAxes, fontsize=7, va="top",
            color="white", weight="bold",
            bbox=dict(facecolor=col, edgecolor="none", pad=2), zorder=10)

    if obstacles:
        for obs in obstacles:
            ax.add_patch(mpatches.Rectangle(
                (obs.x_min, obs.y_min), obs.length, obs.width,
                facecolor="#888888", edgecolor="black", lw=1.5,
                alpha=0.75, zorder=3))

    patches, arrows, id_labels = [], [], []
    for _ in range(n_agv):
        rect = mpatches.FancyBboxPatch(
            (-AGV_LENGTH / 2, -AGV_WIDTH / 2), AGV_LENGTH, AGV_WIDTH,
            boxstyle="round,pad=0.05",
            facecolor=_COL_UNLOADED, edgecolor=_COL_NORMAL, lw=2.0, zorder=4)
        ax.add_patch(rect)

        arr = ax.annotate(
            "", xy=(0.5, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="-|>", color="white",
                            lw=1.0, mutation_scale=8),
            zorder=5)

        lbl = ax.text(0, 0, "", ha="center", va="center",
                      fontsize=6, color="white", weight="bold", zorder=6)
        patches.append(rect)
        arrows.append(arr)
        id_labels.append(lbl)

    time_txt   = ax.text(0.01, 0.04, "", transform=ax.transAxes,
                         fontsize=10, va="bottom", color="#222222")
    status_txt = ax.text(0.99, 0.04, "", transform=ax.transAxes,
                         fontsize=10, va="bottom", ha="right", color="#cc0000")

    return patches, arrows, id_labels, time_txt, status_txt


def _update_subplot(patches, arrows, id_labels,
                    time_txt, status_txt,
                    snap, pair_times, t_actual, makespan, overall_mcd):
    for k in range(len(patches)):
        x, y, th, state, arrived, cargo, ty = snap[k]

        tr = (Affine2D().rotate(th).translate(x, y)) + patches[k].axes.transData
        patches[k].set_transform(tr)

        if arrived:
            patches[k].set_facecolor(_COL_ARRIVED)
            patches[k].set_edgecolor(_COL_ARRIVED)
            patches[k].set_alpha(0.35)
        else:
            patches[k].set_facecolor(_COL_LOADED if cargo else _COL_UNLOADED)
            patches[k].set_edgecolor(_COL_DETOUR if abs(ty) > 0.05 else _COL_NORMAL)
            patches[k].set_linewidth(2.0)
            patches[k].set_alpha(0.22 if state == "WAIT" else 0.92)

        hx = x + (AGV_LENGTH / 2 + 0.12) * math.cos(th)
        hy = y + (AGV_LENGTH / 2 + 0.12) * math.sin(th)
        arrows[k].xy = (hx, hy)
        arrows[k].set_position((x, y))
        id_labels[k].set_position((x, y))
        id_labels[k].set_text(str(k))
        id_labels[k].set_visible(not arrived)

    live_mcd = _live_mcd(snap)
    mcd_str  = f"{live_mcd:.3f}m" if live_mcd < 999 else "—"
    time_txt.set_text(
        f"t={t_actual:.1f}s  ms={makespan:.1f}s  MCD={mcd_str}  (min={overall_mcd:.3f}m)")
    status_txt.set_text(_sim_status(snap, pair_times, t_actual))


# ── main GIF builder ───────────────────────────────────────────────────────────
def make_comparison_gif(corridor_width: float,
                        payload_profile: str,
                        n_agv: int = N_AGV,
                        seed: int = SEED,
                        fps: int = FPS,
                        obstacles=None) -> str:

    sim_data = {}
    for algo in ALGO_ORDER:
        _, traj, ms, pt, mcd, _, _ = run_scenario(
            n_agv=n_agv, algorithm=algo, seed=seed,
            corridor_width=corridor_width,
            payload_profile=payload_profile,
            record_trajectory=True,
            obstacles=obstacles,
        )
        sim_data[algo] = dict(traj=traj, makespan=ms, pair_times=pt, mcd=mcd)

    max_t    = max(d["traj"][-1][0] for d in sim_data.values())
    n_frames = min(MAX_FRAMES, max(1, int(max_t / DT) + 1))
    t_steps  = [i * max_t / max(n_frames - 1, 1) for i in range(n_frames)]

    fig, axes = plt.subplots(4, 1, figsize=(32, 16), dpi=DPI)
    fig.patch.set_facecolor("#e8e8e8")
    fig.suptitle(
        f"Algorithm Comparison — corridor={corridor_width}m  "
        f"n_agv={n_agv}  payload={payload_profile}  seed={seed}",
        fontsize=13, weight="bold", y=0.99)

    subplot_elems = {}
    for row, algo in enumerate(ALGO_ORDER):
        ax = axes[row]
        elems = _init_subplot(ax, algo, corridor_width, n_agv, obstacles=obstacles)
        subplot_elems[algo] = elems

    fig.tight_layout(rect=[0, 0, 1, 0.975], pad=0.8, h_pad=0.6, w_pad=0.4)

    def update(fi):
        t_req = t_steps[fi]
        for algo in ALGO_ORDER:
            d = sim_data[algo]
            t_actual, snap = _snap_at(d["traj"], t_req)
            _update_subplot(*subplot_elems[algo],
                            snap, d["pair_times"], t_actual, d["makespan"], d["mcd"])

    anim = FuncAnimation(fig, update, frames=list(range(n_frames)),
                         interval=1000 / fps)

    # naming: w{corridor}_n{n_agv}_{payload}_s{seed}.gif
    fname = f"w{corridor_width}_n{n_agv}_{payload_profile}_s{seed}.gif"
    path  = os.path.join(GIF_DIR, fname)
    anim.save(path, writer=PillowWriter(fps=fps), dpi=DPI)
    plt.close(fig)
    return path


# ── entry point ────────────────────────────────────────────────────────────────
if __name__ == "__main__":
    # Delete all existing GIFs
    existing = glob.glob(os.path.join(GIF_DIR, "*.gif"))
    for f in existing:
        os.remove(f)
    print(f"[+] Deleted {len(existing)} existing GIFs")

    main_configs = [(cw, pp)
                    for cw in CORRIDOR_WIDTHS
                    for pp in PAYLOAD_PROFILES]

    obs_configs  = [(cw, op)
                    for cw in CORRIDOR_WIDTHS
                    for op in OBSTACLE_PROFILES]

    total  = len(main_configs) + len(obs_configs)
    done   = 0
    t_all  = time.time()

    print(f"[+] Generating {len(main_configs)} main + {len(obs_configs)} obstacle "
          f"comparison GIFs (n_agv={N_AGV}; mixed/obstacle use seed "
          f"{SPLIT_CARGO_SEED} for visible cargo colours) ...\n")

    # Main experiment GIFs
    for cw, pp in main_configs:
        t0   = time.time()
        path = make_comparison_gif(cw, pp, n_agv=N_AGV, seed=seed_for(pp))
        done += 1
        size  = os.path.getsize(path) / 1e6
        fname = os.path.basename(path)
        print(f"  [{done:2d}/{total}] {fname}  {size:.1f} MB  ({time.time()-t0:.0f}s)")

    # Obstacle experiment GIFs
    for cw, op in obs_configs:
        t0  = time.time()
        obs = make_obstacle_set(op, cw)
        path = make_comparison_gif(cw, op, n_agv=N_AGV, seed=seed_for(op),
                                   obstacles=obs)
        done += 1
        size  = os.path.getsize(path) / 1e6
        fname = os.path.basename(path)
        print(f"  [{done:2d}/{total}] {fname}  {size:.1f} MB  ({time.time()-t0:.0f}s)")

    print(f"\n[✓] {done} GIFs saved to {GIF_DIR}  (total {time.time()-t_all:.0f}s)")
