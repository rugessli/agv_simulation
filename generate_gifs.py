"""Diagnostic GIF generator for selected configurations."""
from __future__ import annotations
import os, math, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.transforms import Affine2D

from config import AGV_LENGTH, AGV_WIDTH, CORRIDOR_LENGTH, DT
from simulation import run_scenario, make_obstacle_set

GIF_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output", "gifs")
os.makedirs(GIF_DIR, exist_ok=True)

# State → display alpha
STATE_ALPHA = {"RUN": 0.88, "APPROACH": 0.75, "SLOW": 0.60, "CREEP": 0.40, "WAIT": 0.25}


def make_gif(corridor_width: float, n_agv: int,
             payload_profile: str, algorithm: str,
             seed: int = 0, fps: int = 12, max_frames: int = 250,
             obstacles: list = None) -> None:

    if obstacles is None and payload_profile in (
            "obstacle", "obstacle_top", "obstacle_mid", "obstacle_bot"):
        obstacles = make_obstacle_set(payload_profile, corridor_width)

    _, traj, makespan, pair_times, mcd, _, _ = run_scenario(
        n_agv=n_agv, algorithm=algorithm, seed=seed,
        corridor_width=corridor_width,
        payload_profile=payload_profile,
        record_trajectory=True,
        obstacles=obstacles,
    )

    half_w = corridor_width / 2.0
    y_pad  = 0.45
    fig_w  = 18
    fig_h  = fig_w * (2 * (half_w + y_pad)) / CORRIDOR_LENGTH
    fig_h  = max(2.2, min(fig_h, 4.5))

    fig, ax = plt.subplots(figsize=(fig_w, fig_h))
    ax.set_xlim(-0.5, CORRIDOR_LENGTH + 0.5)
    ax.set_ylim(-half_w - y_pad, half_w + y_pad)
    ax.set_aspect("equal")

    title = (f"{algorithm}  |  {n_agv} AGV  |  w={corridor_width} m  |  "
             f"{payload_profile}  |  seed={seed}")
    ax.set_title(title, fontsize=9)
    ax.set_xlabel("x (m)", fontsize=8)
    ax.set_ylabel("y (m)", fontsize=8)
    ax.tick_params(labelsize=7)

    # Walls + centreline
    wall_rect = mpatches.Rectangle(
        (0, -half_w), CORRIDOR_LENGTH, 2 * half_w,
        fill=False, lw=1.5, edgecolor="black")
    ax.add_patch(wall_rect)
    ax.axhline(0, ls="--", lw=0.7, color="grey", alpha=0.45)

    # Draw static obstacles
    if obstacles:
        for obs in obstacles:
            obs_patch = mpatches.Rectangle(
                (obs.x_min, obs.y_min), obs.length, obs.width,
                facecolor="#888888", edgecolor="black", lw=1.0, alpha=0.8, zorder=3)
            ax.add_patch(obs_patch)

    # AGV patches + arrows + labels
    colors = plt.cm.tab10(np.linspace(0, 1, max(n_agv, 4)))
    rects, arrows, txt_labels = [], [], []
    for k in range(n_agv):
        r = mpatches.Rectangle(
            (0, 0), AGV_LENGTH, AGV_WIDTH,
            facecolor=colors[k], edgecolor="black", lw=0.7, alpha=0.88)
        ax.add_patch(r)

        arr = ax.annotate(
            "", xy=(0, 0), xytext=(0, 0),
            arrowprops=dict(arrowstyle="->", color="white", lw=1.1))

        lbl = ax.text(0, 0, str(k), ha="center", va="center",
                      fontsize=7, color="white", weight="bold")

        rects.append(r)
        arrows.append(arr)
        txt_labels.append(lbl)

    # Overlay texts
    time_txt  = ax.text(0.5,  half_w + y_pad - 0.08, "",
                        fontsize=8, va="top")
    state_txt = ax.text(CORRIDOR_LENGTH / 2, half_w + y_pad - 0.08, "",
                        fontsize=7, ha="center", va="top", color="#880000")

    n_raw = len(traj)
    stride = max(1, n_raw // max_frames)
    frames = list(range(0, n_raw, stride))

    art_list = []
    for v in pair_times.values():
        if v["resolved"] is not None:
            art_list.append(v["resolved"] - v["detect"])
    art_str = f"ART={sum(art_list)/len(art_list):.2f}s" if art_list else "ART=n/a"

    def update(fi: int):
        t, snap = traj[fi]
        for k, entry in enumerate(snap):
            x, y, th, state, arrived = entry[:5]
            tr = (Affine2D()
                  .translate(-AGV_LENGTH / 2, -AGV_WIDTH / 2)
                  .rotate(th)
                  .translate(x, y)) + ax.transData
            rects[k].set_transform(tr)
            alpha = 0.15 if arrived else STATE_ALPHA.get(state, 0.88)
            rects[k].set_alpha(alpha)

            txt_labels[k].set_position((x, y))

            hx = x + (AGV_LENGTH / 2 + 0.1) * math.cos(th)
            hy = y + (AGV_LENGTH / 2 + 0.1) * math.sin(th)
            arrows[k].xy = (hx, hy)
            arrows[k].set_position((x, y))

        time_txt.set_text(
            f"t={t:.1f}s   makespan={makespan:.1f}s   MCD={mcd:.3f}m   {art_str}")

        state_parts = [
            f"AGV{k}: {snap[k][3]}" + (" ✓" if snap[k][4] else "")
            for k in range(min(n_agv, len(snap)))
        ]
        state_txt.set_text("  |  ".join(state_parts))

    anim = FuncAnimation(fig, update, frames=frames, interval=1000 / fps)
    fig.tight_layout(pad=0.4)

    fname = f"w{corridor_width}_n{n_agv}_{payload_profile}_{algorithm}_s{seed}.gif"
    path  = os.path.join(GIF_DIR, fname)
    anim.save(path, writer=PillowWriter(fps=fps), dpi=90)
    plt.close(fig)
    print(f"[+] {fname}  ({len(frames)} frames, makespan={makespan:.1f}s, "
          f"MCD={mcd:.3f}m, {art_str})")


# ──────────────────────────────────────────────
# Configuration list
# ──────────────────────────────────────────────
CONFIGS = [
    # (corridor_width, n_agv, payload_profile, algorithm)
    # Smoke-test: 2-AGV (Fix 1 speed verification)
    (3.2, 2, "mixed", "DLTO"),
    (3.2, 2, "mixed", "ORCA"),
    (3.2, 2, "mixed", "SW"),
    (3.2, 2, "mixed", "FLC_ORCA"),
    # Multi-AGV (Fix 3+4 verification)
    (3.2, 3, "mixed", "DLTO"),
    (3.2, 3, "mixed", "ORCA"),
    (3.2, 4, "mixed", "DLTO"),
    (3.2, 4, "mixed", "ORCA"),
    # Narrowest corridor (stress test)
    (2.8, 2, "mixed", "DLTO"),
    (2.8, 3, "mixed", "DLTO"),
    (2.8, 4, "mixed", "DLTO"),
    (2.8, 4, "mixed", "SW"),
    # Obstacle GIFs — 3 positions × 3 widths × 4 algorithms
    (2.8, 2, "obstacle_top", "DLTO"),
    (2.8, 2, "obstacle_mid", "DLTO"),
    (2.8, 2, "obstacle_bot", "DLTO"),
    (3.2, 2, "obstacle_top", "DLTO"),
    (3.2, 2, "obstacle_mid", "DLTO"),
    (3.2, 2, "obstacle_bot", "DLTO"),
    (3.6, 2, "obstacle_top", "DLTO"),
    (3.6, 2, "obstacle_mid", "DLTO"),
    (3.6, 2, "obstacle_bot", "DLTO"),
    (3.2, 2, "obstacle_top", "ORCA"),
    (3.2, 2, "obstacle_mid", "ORCA"),
    (3.2, 2, "obstacle_bot", "ORCA"),
    (3.2, 2, "obstacle_top", "FLC_ORCA"),
    (3.2, 2, "obstacle_mid", "FLC_ORCA"),
    (3.2, 2, "obstacle_bot", "FLC_ORCA"),
    (3.2, 2, "obstacle_top", "SW"),
    (3.2, 2, "obstacle_mid", "SW"),
    (3.2, 2, "obstacle_bot", "SW"),
]

if __name__ == "__main__":
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", type=int, default=None,
                    help="Run only config index (1-based)")
    ap.add_argument("--smoke", action="store_true",
                    help="Run first 8 configs only (smoke test)")
    args = ap.parse_args()

    if args.only is not None:
        idx = args.only - 1
        w, n, p, a = CONFIGS[idx]
        print(f"[>] Config {args.only}: w={w} n={n} {p} {a}")
        make_gif(w, n, p, a)
    elif args.smoke:
        for i, (w, n, p, a) in enumerate(CONFIGS[:8], 1):
            print(f"[>] Config {i}/8: w={w} n={n} {p} {a}")
            make_gif(w, n, p, a)
    else:
        for i, (w, n, p, a) in enumerate(CONFIGS, 1):
            print(f"[>] Config {i}/{len(CONFIGS)}: w={w} n={n} {p} {a}")
            make_gif(w, n, p, a)
    print(f"\n[✓] GIFs saved to {GIF_DIR}")
