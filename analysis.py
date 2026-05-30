"""
analysis.py
===========
Post-simulation visualisation: GIF animations and metric bar charts.

What it does
------------
  * `animate_side_by_side(traj_dlto, traj_sw, ...)` — single GIF showing
        DLTO (top) and SW (bottom) for the same scenario, with corridor
        walls, AGV OBBs, heading arrows, and a t = ... s caption.
  * `plot_metric_bars(agg, save_path)` — 5-panel bar chart comparing the
        SW and DLTO summary numbers across scenarios.
  * `save_csv(df, path)` — raw per-replication CSV dump.

ROS2 migration notes
--------------------
ROS2-READY :
  - All functions in this file: they consume in-memory data (pandas or
    lists of poses).  A ROS2 bag → DataFrame converter is all that is
    needed to reuse this analysis post-mission.

ROS2-REPLACE :
  - none.
"""

from __future__ import annotations
import math
import os
from typing import List, Tuple

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation, PillowWriter
from matplotlib.transforms import Affine2D

from agv_model import AGV_LENGTH, AGV_WIDTH
from simulation import CORRIDOR_LENGTH, HALF_W, DT


# ---------------------------------------------------------------------------
# Animation — side-by-side DLTO vs SW
# ---------------------------------------------------------------------------
def animate_side_by_side(traj_dlto: List, traj_sw: List,
                         n_agv: int, scenario_label: str,
                         save_path: str,
                         obstacles=None,
                         events_dlto=None,
                         events_sw=None) -> None:

    obstacles   = obstacles   or []
    events_dlto = events_dlto or []
    events_sw   = events_sw   or []

    # Align lengths — pad the shorter trajectory with its final frame
    L = max(len(traj_dlto), len(traj_sw))
    def _pad(traj):
        if len(traj) >= L: return traj
        last = traj[-1]
        return traj + [last] * (L - len(traj))
    traj_dlto = _pad(traj_dlto)
    traj_sw   = _pad(traj_sw)

    # Fixed corridor view: y axis exactly ±0.75 with a tiny label margin.
    max_y = HALF_W + 0.10

    fig, (ax_d, ax_s) = plt.subplots(2, 1, figsize=(12, 6))
    for ax, title in ((ax_d, f"DLTO — {scenario_label}"),
                      (ax_s, f"SW   — {scenario_label}")):
        ax.set_xlim(-0.5, CORRIDOR_LENGTH + 0.5)
        ax.set_ylim(-max_y, max_y)
        ax.set_aspect("equal")
        ax.set_title(title)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        # solid corridor walls
        ax.add_patch(patches.Rectangle((0, -HALF_W), CORRIDOR_LENGTH,
                                       2 * HALF_W, fill=False, lw=1.5))
        # dashed centre line — lane divider
        ax.axhline(0, ls="--", lw=1.0, color="grey")
        # static obstacles (filled grey + label)
        for o in obstacles:
            ax.add_patch(patches.Rectangle(
                (o.x_min, o.y_min), o.length, o.width,
                facecolor="dimgray", edgecolor="black", lw=1.0))
            ax.text(o.cx, o.cy, "OBS", ha="center", va="center",
                    fontsize=8, color="white", weight="bold")

    colors = plt.cm.tab10(np.linspace(0, 1, max(n_agv, 3)))

    def _build_axis(ax):
        rects, arrows, labels = [], [], []
        for k in range(n_agv):
            r = patches.Rectangle((0, 0), AGV_LENGTH, AGV_WIDTH,
                                  facecolor=colors[k], edgecolor="black",
                                  lw=1.0, alpha=0.85)
            ax.add_patch(r)
            arrow = ax.annotate("", xy=(0, 0), xytext=(0, 0),
                                arrowprops=dict(arrowstyle="->",
                                                color="white", lw=1.4))
            txt = ax.text(0, 0, str(k), ha="center", va="center",
                          fontsize=8, color="white", weight="bold")
            rects.append(r); arrows.append(arrow); labels.append(txt)
        return rects, arrows, labels

    d_rects, d_arrows, d_labels = _build_axis(ax_d)
    s_rects, s_arrows, s_labels = _build_axis(ax_s)

    d_time = ax_d.text(0.1, max_y - 0.15, "", fontsize=10)
    s_time = ax_s.text(0.1, max_y - 0.15, "", fontsize=10)
    d_event = ax_d.text(CORRIDOR_LENGTH / 2, max_y - 0.15, "",
                        fontsize=10, color="red", ha="center",
                        weight="bold")
    s_event = ax_s.text(CORRIDOR_LENGTH / 2, max_y - 0.15, "",
                        fontsize=10, color="red", ha="center",
                        weight="bold")

    def _event_text(events, t):
        """Most recent event up to time t, fading from red→green when resolved."""
        msg = ""
        for et, em in events:
            if et <= t:
                msg = em
            else:
                break
        return msg

    def _place(rects, arrows, labels, snap):
        for k, (x, y, th, state, arrived) in enumerate(snap):
            tr = (Affine2D()
                  .translate(-AGV_LENGTH / 2, -AGV_WIDTH / 2)
                  .rotate(th).translate(x, y)) + rects[k].axes.transData
            rects[k].set_transform(tr)
            rects[k].set_alpha(0.2 if arrived else (0.4 if state == "WAIT" else 0.9))
            labels[k].set_position((x, y))
            # heading arrow inside the body
            hx = x + 0.35 * math.cos(th); hy = y + 0.35 * math.sin(th)
            arrows[k].xy = (hx, hy); arrows[k].set_position((x, y))

    def draw(frame_idx):
        t_d, snap_d = traj_dlto[frame_idx]
        t_s, snap_s = traj_sw[frame_idx]
        _place(d_rects, d_arrows, d_labels, snap_d)
        _place(s_rects, s_arrows, s_labels, snap_s)
        d_time.set_text(f"t = {t_d:5.2f} s")
        s_time.set_text(f"t = {t_s:5.2f} s")
        msg_d = _event_text(events_dlto, t_d)
        msg_s = _event_text(events_sw,   t_s)
        d_event.set_text(msg_d)
        s_event.set_text(msg_s)
        d_event.set_color("green" if "RESOLVED" in msg_d else "red")
        s_event.set_color("green" if "RESOLVED" in msg_s else "red")

    # ~20 fps
    stride = max(1, int(round(1.0 / (20 * DT))))
    frames = list(range(0, L, stride))
    anim = FuncAnimation(fig, draw, frames=frames, interval=1000 / 20)
    fig.tight_layout()
    anim.save(save_path, writer=PillowWriter(fps=20), dpi=100)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Metric bar chart — DLTO vs SW × scenarios
# ---------------------------------------------------------------------------
def plot_metric_bars(agg: pd.DataFrame, save_path: str) -> None:
    metric_defs = [
        ("DRR_mean", "DRR",   "Deadlock Resolution Rate"),
        ("ART_mean", "ART",   "Average Resolution Time (s)"),
        ("ST_mean",  "ST",    "Throughput (AGV / 10-min)"),
        ("MCD_mean", "MCD",   "Min Clearance (m)"),
        ("NEC_norm", "NEC",   "Normalised Energy (DLTO / SW)"),
    ]
    fig, axes = plt.subplots(1, 5, figsize=(22, 4.4))
    scenarios = sorted(agg["scenario"].unique())
    width = 0.35
    xpos = np.arange(len(scenarios))

    def _get(scn, algo, key):
        row = agg[(agg.scenario == scn) & (agg.algorithm == algo)]
        return float(row[key].values[0]) if len(row) else float("nan")

    for ax, (col, short, title) in zip(axes, metric_defs):
        sw  = [_get(s, "SW",   col) for s in scenarios]
        dl  = [_get(s, "DLTO", col) for s in scenarios]
        b1 = ax.bar(xpos - width / 2, sw, width, label="SW",   color="#888")
        b2 = ax.bar(xpos + width / 2, dl, width, label="DLTO", color="#1f77b4")
        ax.set_xticks(xpos); ax.set_xticklabels(scenarios)
        ax.set_title(f"{short} — {title}")
        ax.grid(axis="y", alpha=0.3)
        for bars in (b1, b2):
            for b in bars:
                h = b.get_height()
                if h == h:        # not NaN
                    ax.text(b.get_x() + b.get_width() / 2, h, f"{h:.2f}",
                            ha="center", va="bottom", fontsize=8)
        ax.legend(fontsize=8)
    fig.suptitle("DLTO vs Stop-and-Wait — 80-replication summary", fontsize=13)
    fig.tight_layout(rect=[0, 0, 1, 0.95])
    fig.savefig(save_path, dpi=130)
    plt.close(fig)


# ---------------------------------------------------------------------------
# Trivial CSV writer
# ---------------------------------------------------------------------------
def save_csv(df: pd.DataFrame, path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    df.to_csv(path, index=False)
