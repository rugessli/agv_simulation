"""Fig. 1: schematic of the bidirectional AGV corridor (dimensions in mm, drawn to scale)."""
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle, FancyArrowPatch, Circle

FS = 8                                   # single font size for all labels (pt)
plt.rcParams.update({"font.family": "DejaVu Sans", "font.size": FS})
W_PHYS, W_NAV = 3386.0, 3200.0           # corridor widths (Section 1.1)
L_AGV, W_AGV = 2350.0, 1100.0            # AGV body (Section 3.1)
X0, X1 = 0.0, 17000.0                    # drawn corridor segment
y_lo, y_hi = -W_PHYS / 2, W_PHYS / 2
m = (W_PHYS - W_NAV) / 2                 # band between wall line and floor marking
n_lo, n_hi = y_lo + m, y_hi - m          # navigable boundaries
ROW = 1700.0
ink, grey, light, yel, blue = "#222222", "#9a9a9a", "#e9e9e9", "#c9a400", "#1f3f66"

fig, ax = plt.subplots(figsize=(6.85, 2.9))

# upper row: stranding machines with reel storage in front
for i, x in enumerate(range(300, 16000, 4200)):
    ax.add_patch(Rectangle((x, y_hi + 700), 3400, ROW - 700, fc=light, ec=ink, lw=0.7))
    ax.add_patch(Rectangle((x + 550, y_hi), 2600, 700, fc="white", ec=grey, lw=0.6, hatch="////"))
    if i == 0:
        ax.text(x + 1850, y_hi + 700 + (ROW - 700) / 2, "stranding\nmachine", ha="center", va="center", linespacing=1.0)
        ax.text(x + 1850, y_hi + 350, "reel storage", ha="center", va="center",
                bbox=dict(fc="white", ec="none", pad=0.4))

# lower row: copper cable reel buffer storage
for i, x in enumerate(range(700, 16000, 4200)):
    ax.add_patch(Rectangle((x, y_lo - ROW), 2800, ROW - 250, fc="white", ec=grey, lw=0.6, hatch="////"))
    if i == 0:
        ax.text(x + 1400, y_lo - 250 - (ROW - 250) / 2, "reel buffer", ha="center", va="center",
                bbox=dict(fc="white", ec="none", pad=0.4))

# corridor: wall lines (black), floor markings = navigable boundary (yellow dashed), centreline
XE = 19800                               # extension lines run to the dimension arrows
for y in (y_lo, y_hi):
    ax.plot([X0, X1], [y, y], color=ink, lw=1.0)
    ax.plot([X1, 17700], [y, y], color=ink, lw=0.5)
for y in (n_lo, n_hi):
    ax.plot([X0, X1], [y, y], color=yel, lw=1.0, ls=(0, (6, 3)))
    ax.plot([X1, XE], [y, y], color=yel, lw=0.5)
ax.plot([X0, X1], [0, 0], color=grey, lw=0.5, ls=(0, (2, 3)))

# two AGVs approaching head-on
def agv(xc, heading, loaded):
    ax.add_patch(Rectangle((xc - L_AGV / 2, -W_AGV / 2), L_AGV, W_AGV, fc="#4c78a8" if loaded else "white",
                           ec=blue, lw=1.0, zorder=3))
    if loaded:
        ax.add_patch(Circle((xc, 0), 380, fc="white", ec=blue, lw=0.8, zorder=4))
        ax.add_patch(Circle((xc, 0), 120, fc=blue, ec="none", zorder=5))
    ax.add_patch(FancyArrowPatch((xc + heading * (L_AGV / 2 + 80), 0), (xc + heading * (L_AGV / 2 + 900), 0),
                                 arrowstyle="-|>", mutation_scale=9, lw=1.0, color=ink, zorder=3))
agv(4300, +1, True)
agv(11200, -1, False)
ax.text(4300, -W_AGV / 2 - 120, "loaded AGV", ha="center", va="top")
ax.text(11200, -W_AGV / 2 - 120, "unloaded AGV", ha="center", va="top")

# AGV body dimensions
xa = 4300 - L_AGV / 2
ax.add_patch(FancyArrowPatch((xa, W_AGV / 2 + 300), (xa + L_AGV, W_AGV / 2 + 300), arrowstyle="<|-|>",
                             mutation_scale=6, lw=0.6, color="#555555"))
ax.text(xa + L_AGV / 2, W_AGV / 2 + 380, "2,350 mm", ha="center", va="bottom", color="#333333")
ax.add_patch(FancyArrowPatch((xa - 300, -W_AGV / 2), (xa - 300, W_AGV / 2), arrowstyle="<|-|>",
                             mutation_scale=6, lw=0.6, color="#555555"))
ax.text(xa - 380, 0, "1,100 mm", rotation=90, ha="right", va="center", color="#333333")

# corridor width dimensions: 3,386 between wall lines, 3,200 between floor markings
def vdim(x, ya, yb, label, ylab):
    ax.add_patch(FancyArrowPatch((x, ya), (x, yb), arrowstyle="<|-|>", mutation_scale=7, lw=0.8, color=ink,
                                 shrinkA=0, shrinkB=0))
    ax.text(x + 150, ylab, label, ha="left", va="center", linespacing=1.1)
vdim(17500, y_lo, y_hi, "3,386 mm\n(physical)", 650)
vdim(19700, n_lo, n_hi, "3,200 mm\n(navigable)", -650)

# scale bar (1 m)
sb_y = y_lo - ROW - 500
ax.plot([700, 1700], [sb_y, sb_y], color=ink, lw=1.5, solid_capstyle="butt")
for xx in (700, 1700):
    ax.plot([xx, xx], [sb_y - 90, sb_y + 90], color=ink, lw=0.8)
ax.text(1850, sb_y, "1 m", ha="left", va="center")

# legend for line types
lx, ly = 7200, sb_y
ax.plot([lx, lx + 900], [ly, ly], color=ink, lw=1.0); ax.text(lx + 1050, ly, "wall line", va="center")
ax.plot([lx + 3000, lx + 3900], [ly, ly], color=yel, lw=1.0, ls=(0, (6, 3)))
ax.text(lx + 4050, ly, "floor marking (navigable boundary)", va="center")

ax.set_xlim(X0 - 1200, 22000)
ax.set_ylim(sb_y - 300, y_hi + ROW + 150)
ax.set_aspect("equal"); ax.axis("off")
fig.tight_layout(pad=0.2)
for ext in ("png", "tiff", "pdf"):
    fig.savefig(f"fig1_corridor.{ext}", dpi=600, **({"pil_kwargs": {"compression": "tiff_lzw"}} if ext == "tiff" else {}))
print("saved")
