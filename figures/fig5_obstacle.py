"""Fig. 5: obstacle experiment, drawn at print size (157 mm wide), all text 8 pt."""
import sys, os
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np, pandas as pd
CSV = sys.argv[1] if len(sys.argv) > 1 else "output/results_obstacle.csv"
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": 8, "axes.titlesize": 8,
                     "axes.labelsize": 8, "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 8,
                     "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6})
ALGOS = ["DLTO", "SW", "ORCA", "FLC_ORCA"]; LAB = {"FLC_ORCA": "FLC-ORCA"}
COL = {"DLTO": "#e67e22", "SW": "#7f8c8d", "ORCA": "#8e44ad", "FLC_ORCA": "#16a085"}
POS = ["obstacle_top", "obstacle_mid", "obstacle_bot"]; PL = {"obstacle_top": "Top", "obstacle_mid": "Middle", "obstacle_bot": "Bottom"}
W = [2.8, 3.2, 3.6]
df = pd.read_csv(CSV)
g = df.groupby(["payload_profile", "corridor_width", "algorithm"]).mean(numeric_only=True)
def cell(p, w, a, c): return float(g.loc[(p, w, a), c])
def nec(p, w, a): return cell(p, w, a, "NEC_raw") / cell(p, w, "SW", "NEC_raw")
panels = [("(a) DRR", "DRR", lambda p, w, a: cell(p, w, a, "DRR")),
          ("(b) ART (s)", "ART", lambda p, w, a: cell(p, w, a, "ART")),
          ("(c) ST (missions / 10 min)", "ST", lambda p, w, a: cell(p, w, a, "ST")),
          ("(d) MCD (m), SW not shown", "MCD", lambda p, w, a: cell(p, w, a, "MCD")),
          ("(e) NEC (relative to SW)", "NEC", nec)]
groups = [(p, w) for p in POS for w in W]
x = np.arange(len(groups)); bw = 0.2
fig, axs = plt.subplots(3, 2, figsize=(157 / 25.4, 6.6))
for ax, (title, key, fn) in zip(axs.flat, panels):
    for k, a in enumerate(ALGOS):
        if key == "MCD" and a == "SW": continue
        ax.bar(x + (k - 1.5) * bw, [fn(p, w, a) for p, w in groups], bw, color=COL[a], edgecolor="black", lw=0.3)
    ax.axvspan(3 - 0.5, 3 + 0.5, color="#cccccc", alpha=0.45, zorder=0)       # middle obstacle, 2.8 m
    for b in (2.5, 5.5): ax.axvline(b, color="#bbbbbb", lw=0.5)
    ax.set_xticks(x); ax.set_xticklabels([str(w) for _, w in groups])
    ax.tick_params(axis="x", length=0, pad=2)
    for j, p in enumerate(POS):
        ax.text(3 * j + 1, -0.13, PL[p], transform=ax.get_xaxis_transform(), ha="center", va="top")
    ax.set_title(title, pad=3); ax.grid(axis="y", ls=":", lw=0.5, alpha=0.6); ax.set_axisbelow(True)
    ax.set_xlim(-0.6, len(groups) - 0.4)
    for s in ("top", "right"): ax.spines[s].set_visible(False)
leg = axs.flat[5]; leg.axis("off")
leg.legend(handles=[Patch(fc=COL[a], ec="black", lw=0.3, label=LAB.get(a, a)) for a in ALGOS]
           + [Patch(fc="#cccccc", alpha=0.45, label="impassable\n(middle, 2.8 m)")],
           loc="center", frameon=False, title="x axis: corridor width (m),\ngrouped by obstacle position", title_fontsize=8)
fig.tight_layout(pad=0.3, h_pad=1.2, w_pad=1.5)
for ext in ("pdf", "png"): fig.savefig(f"Fig5.{ext}", dpi=600)
from PIL import Image
Image.open("Fig5.png").convert("RGB").save("Fig5.tiff", dpi=(600, 600), compression="tiff_lzw")
print("saved", Image.open("Fig5.png").size)
