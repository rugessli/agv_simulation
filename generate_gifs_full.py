"""Generate 9 comparison GIFs — 3 corridor widths × 3 payload profiles.

Fixed: n_agv=3, seed=0.
Layout: 2×2 subplots (DLTO | SW // ORCA | FLC_ORCA), time-synchronised.
Output: output/gifs/comparison_{corridor_width}m_{payload_profile}.gif
"""
from __future__ import annotations
import os
import sys
import math

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.transforms import Affine2D

from config import (
    AGV_LENGTH, AGV_WIDTH, CORRIDOR_LENGTH, DT,
    DETECT_RANGE, D_MIN,
    CORRIDOR_WIDTHS, PAYLOAD_PROFILES,
)
from simulation import run_scenario, pairwise_min_obb_distance

GIF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "gifs")
os.makedirs(GIF_DIR, exist_ok=True)

# ── fixed experiment parameters ───────────────────────────────────────────────
N_AGV      = 3
SEED       = 0
FPS        = 10
MAX_FRAMES = 180          # cap per GIF
DPI        = 80           # 32×80=2560 px wide — balanced quality/size for 40 m corridor

# 4×1 subplot mapping: row → algorithm
LAYOUT = {
    0: "DLTO",
    1: "SW",
    2: "ORCA",
    3: "FLC_ORCA",
}
ALGO_ORDER = ["DLTO", "SW", "ORCA", "FLC_ORCA"]

# ── visual constants ──────────────────────────────────────────────────────────
_COL_LOADED   = "#2ecc71"   # green  — loaded body
_COL_UNLOADED = "#3498db"   # blue   — unloaded body
_COL_ARRIVED  = "#bbbbbb"   # grey   — arrived
_COL_DETOUR   = "#e74c3c"   # red    — edge when detouring
_COL_NORMAL   = "#2c3e50"   # dark   — normal edge
_ALGO_COL = {
    "DLTO":     "#e67e22",
    "SW":       "#c0392b",
    "ORCA":     "#8e44ad",
    "FLC_ORCA": "#16a085",
}


# ── geometry helpers ──────────────────────────────────────────────────────────
def _obb_corners(x, y, theta):
    c, s = math.cos(theta), math.sin(theta)
    l2, w2 = AGV_LENGTH / 2, AGV_WIDTH / 2
    pts = [(l2, w2), (l2, -w2), (-l2, -w2), (-l2, w2)]
    return [(x + lx * c - ly * s, y + lx * s + ly * c) for lx, ly in pts]


def _live_mcd(snap):
    """Minimum OBB clearance between counter-direction live pairs."""
    live = [(i, e) for i, e in enumerate(snap) if not e[4]]
    best = math.inf
    for i, ei in live:
        for j, ej in live:
            if j <= i:
                continue
            # Check opposite direction: heading difference > 90°
            dth = abs((ei[2] - ej[2] + math.pi) % (2 * math.pi) - math.pi)
            if dth < math.radians(90):
                continue
            ca = _obb_corners(ei[0], ei[1], ei[2])
            cb = _obb_corners(ej[0], ej[1], ej[2])
            d = pairwise_min_obb_distance(ca, cb)
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
    """Return trajectory snapshot (t, snap) for the nearest t ≥ t_req."""
    for ft, snap in traj:
        if ft >= t_req:
            return ft, snap
    return traj[-1]


# ── per-subplot initialisation ────────────────────────────────────────────────
def _init_subplot(ax, algorithm, corridor_width, n_agv, payload_profile,
                  obstacles=None):
    half_w = corridor_width / 2.0

    ax.set_xlim(-0.5, CORRIDOR_LENGTH + 0.5)
    ax.set_ylim(-(corridor_width / 2 + 0.2), corridor_width / 2 + 0.2)
    ax.set_aspect("equal")
    ax.set_facecolor("#f5f5f5")
    ax.tick_params(labelsize=7)
    ax.set_xlabel("x (m)", fontsize=9)

    # Corridor walls
    ax.add_patch(mpatches.Rectangle(
        (0, -half_w), CORRIDOR_LENGTH, 2 * half_w,
        fill=False, lw=1.4, edgecolor="black", zorder=3))
    ax.axhline(0, ls="--", lw=0.5, color="#aaaaaa", alpha=0.5, zorder=1)

    # Algorithm badge
    col = _ALGO_COL[algorithm]
    ax.text(0.01, 0.96, f"  {algorithm}  ",
            transform=ax.transAxes, fontsize=7, va="top",
            color="white", weight="bold",
            bbox=dict(facecolor=col, edgecolor="none", pad=2), zorder=10)

    # AGV patches + arrows + id labels
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
                      fontsize=5, color="white", weight="bold", zorder=6)

        patches.append(rect)
        arrows.append(arr)
        id_labels.append(lbl)

    # Dynamic text overlays
    time_txt   = ax.text(0.01, 0.04, "", transform=ax.transAxes,
                         fontsize=10, va="bottom", color="#222222")
    status_txt = ax.text(0.99, 0.04, "", transform=ax.transAxes,
                         fontsize=10, va="bottom", ha="right", color="#cc0000")

    # Draw static obstacles as grey rectangles
    if obstacles:
        for obs in obstacles:
            ax.add_patch(mpatches.Rectangle(
                (obs.x_min, obs.y_min), obs.length, obs.width,
                facecolor="#888888", edgecolor="black", lw=1.5,
                alpha=0.75, zorder=3))

    return patches, arrows, id_labels, time_txt, status_txt


def _update_subplot(patches, arrows, id_labels,
                    time_txt, status_txt,
                    snap, pair_times, t_actual,
                    makespan, overall_mcd):
    n = len(patches)
    for k in range(n):
        x, y, th, state, arrived, cargo, ty = snap[k]
        c = math.cos(th)

        tr = (Affine2D().rotate(th).translate(x, y)) + patches[k].axes.transData
        patches[k].set_transform(tr)

        if arrived:
            patches[k].set_facecolor(_COL_ARRIVED)
            patches[k].set_edgecolor(_COL_ARRIVED)
            patches[k].set_alpha(0.35)
        else:
            patches[k].set_facecolor(_COL_LOADED if cargo else _COL_UNLOADED)
            detouring = abs(ty) > 0.05
            patches[k].set_edgecolor(_COL_DETOUR if detouring else _COL_NORMAL)
            patches[k].set_linewidth(2.0)
            patches[k].set_alpha(0.22 if state == "WAIT" else 0.92)

        # Direction arrow
        hx = x + (AGV_LENGTH / 2 + 0.12) * c
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


# ── main GIF builder ──────────────────────────────────────────────────────────
def make_comparison_gif(corridor_width: float,
                        payload_profile: str,
                        n_agv: int = N_AGV,
                        seed: int = SEED,
                        fps: int = FPS,
                        obstacles=None) -> str:

    # ── run all 4 simulations ────────────────────────────────────────────────
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

    # ── compute shared time axis ─────────────────────────────────────────────
    max_t     = max(d["traj"][-1][0] for d in sim_data.values())
    n_frames  = min(MAX_FRAMES, max(1, int(max_t / DT) + 1))
    t_steps   = [i * max_t / max(n_frames - 1, 1) for i in range(n_frames)]

    # ── build figure ─────────────────────────────────────────────────────────
    fig, axes = plt.subplots(4, 1, figsize=(32, 16), dpi=DPI)
    fig.patch.set_facecolor("#e8e8e8")
    fig.suptitle(
        f"Algorithm Comparison — corridor={corridor_width}m  "
        f"n_agv={n_agv}  payload={payload_profile}  seed={seed}",
        fontsize=13, weight="bold", y=0.99)

    # Map row → algorithm  and  initialise each subplot
    subplot_elems = {}
    for row, algo in LAYOUT.items():
        ax = axes[row]
        patches, arrows, id_labels, time_txt, status_txt = _init_subplot(
            ax, algo, corridor_width, n_agv, payload_profile, obstacles=obstacles)
        subplot_elems[algo] = (patches, arrows, id_labels, time_txt, status_txt)

    fig.tight_layout(rect=[0, 0, 1, 0.975], pad=0.8, h_pad=0.6, w_pad=0.4)

    # ── animation update ─────────────────────────────────────────────────────
    def update(fi):
        t_req = t_steps[fi]
        for algo in ALGO_ORDER:
            d = sim_data[algo]
            t_actual, snap = _snap_at(d["traj"], t_req)
            patches, arrows, id_labels, time_txt, status_txt = subplot_elems[algo]
            _update_subplot(
                patches, arrows, id_labels, time_txt, status_txt,
                snap, d["pair_times"], t_actual, d["makespan"], d["mcd"])

    anim = FuncAnimation(fig, update,
                         frames=list(range(n_frames)),
                         interval=1000 / fps)

    fname = f"comparison_{corridor_width}m_{payload_profile}.gif"
    path  = os.path.join(GIF_DIR, fname)
    anim.save(path, writer=PillowWriter(fps=fps), dpi=DPI)
    plt.close(fig)
    return path


# ── entry point ───────────────────────────────────────────────────────────────
if __name__ == "__main__":
    import argparse, time

    ap = argparse.ArgumentParser(
        description="Generate 9 comparison GIFs (3 widths × 3 payloads, n_agv=3, seed=0)")
    ap.add_argument("--n_agv", type=int, default=N_AGV)
    ap.add_argument("--seed",  type=int, default=SEED)
    args = ap.parse_args()

    configs = [(cw, pp)
               for cw in CORRIDOR_WIDTHS
               for pp in PAYLOAD_PROFILES]

    print(f"[+] Generating {len(configs)} comparison GIFs  "
          f"(n_agv={args.n_agv}, seed={args.seed}) …\n")

    t_total = time.time()
    for idx, (cw, pp) in enumerate(configs, 1):
        t0   = time.time()
        path = make_comparison_gif(cw, pp, n_agv=args.n_agv, seed=args.seed)
        dt   = time.time() - t0
        fname = os.path.basename(path)
        size  = os.path.getsize(path) / 1e6
        print(f"  [{idx:2d}/{len(configs)}] {fname}  {size:.1f} MB  ({dt:.0f}s)")

    print(f"\n[✓] Done in {time.time()-t_total:.0f}s — saved to {GIF_DIR}")
