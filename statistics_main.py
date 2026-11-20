"""Statistical analysis of the main experiment (paper Sections 6.1-6.7, Tables 4-7).

Reads output/results.csv (python main.py) and prints the per-condition means,
the paired Wilcoxon signed-rank tests and Cohen's d_z between algorithms, and
the effect of fleet size. With --encounters it also counts the 3-4 AGV runs in
which a vehicle faced two opposite vehicles within d_detect at the same time.
"""
import os, sys, math, itertools
import pandas as pd
from scipy.stats import wilcoxon

HERE = os.path.dirname(os.path.abspath(__file__))
df = pd.read_csv(os.path.join(HERE, "output", "results.csv"))
A = ["DLTO", "ORCA", "FLC_ORCA", "SW"]; key = ["corridor_width", "n_agv", "payload_profile", "seed"]
pd.set_option("display.width", 200)
print("DRR\n", df.groupby("algorithm").DRR.agg(["mean", "std"]).round(3).T[A].to_string())
t = df.pivot_table(index="algorithm", columns="corridor_width", values="ART", aggfunc="mean").round(2)
t["overall"] = df.groupby("algorithm").ART.mean().round(2); t["sd"] = df.groupby("algorithm").ART.std().round(2)
print("\nART\n", t.loc[A].to_string())
print("\nST\n", df.groupby("algorithm").ST.agg(["mean", "std"]).round(2).T[A].to_string())
print("\nNEC\n", df.groupby("algorithm").NEC_raw.agg(["mean", "std"]).round(3).T[A].to_string())
m = df[df.algorithm != "SW"].pivot_table(index=["corridor_width", "payload_profile"], columns="algorithm", values="MCD", aggfunc="mean")
s = df[df.algorithm != "SW"].pivot_table(index=["corridor_width", "payload_profile"], columns="algorithm", values="MCD", aggfunc="std")
print("\nMCD mean (sd)\n", (m.round(4).astype(str) + " (" + s.round(3).astype(str) + ")")[["DLTO", "ORCA", "FLC_ORCA"]].to_string())
print("overall MCD\n", df.groupby("algorithm").MCD.agg(["mean", "std", "min"]).round(4).T[A].to_string())
w = df.pivot_table(index=key, columns="algorithm", values=["MCD", "ART", "ST", "NEC_raw"]); I = w.index

def cmp(metric, a, b, mask=None, lab=""):
    x, y = w[metric][a], w[metric][b]
    if mask is not None: x, y = x[mask], y[mask]
    d = x - y; nz = d[d.abs() > 1e-12]
    p = wilcoxon(nz).pvalue if len(nz) > 0 else 1.0
    dz = d.mean() / d.std(ddof=1) if d.std(ddof=1) > 0 else float("nan")
    print(f"{lab:16s}{metric:7s} {a} {x.mean():.4f} vs {b} {y.mean():.4f}  {100 * (x.mean() / y.mean() - 1):+.2f}%  p={p:.2g}  dz={dz:+.2f}  n={len(d)}")

print("\nPaired comparisons")
for wd in (3.2, 3.6):
    mk = (I.get_level_values(0) == wd) & (I.get_level_values(2) == "mixed")
    cmp("MCD", "DLTO", "ORCA", mk, f"mixed {wd}"); cmp("MCD", "DLTO", "FLC_ORCA", mk, f"mixed {wd}")
for pp in ("mixed", "unloaded", "loaded"):
    mk = I.get_level_values(2) == pp
    cmp("MCD", "DLTO", "ORCA", mk, pp); cmp("MCD", "DLTO", "FLC_ORCA", mk, pp)
cmp("MCD", "DLTO", "ORCA", None, "overall"); cmp("MCD", "DLTO", "FLC_ORCA", None, "overall")
for mt in ("ART", "ST", "NEC_raw"):
    cmp(mt, "DLTO", "ORCA", None, "overall"); cmp(mt, "DLTO", "FLC_ORCA", None, "overall")
print("\nFleet size, 3.6 m mixed")
for n in (2, 3, 4):
    mk = (I.get_level_values(0) == 3.6) & (I.get_level_values(1) == n) & (I.get_level_values(2) == "mixed")
    cmp("MCD", "DLTO", "ORCA", mk, f"n={n}")

if "--encounters" in sys.argv:
    sys.path.insert(0, HERE)
    import simulation as S
    from config import DETECT_RANGE
    from multiprocessing import Pool
    def job(t):
        wd, n, p, alg, seed = t
        r = S.run_scenario(n_agv=n, algorithm=alg, seed=seed, corridor_width=wd, payload_profile=p, record_trajectory=True)
        dirs = [a.direction for a in r[0]]
        for _, st in r[1]:
            for i, s_ in enumerate(st):
                if s_[4]: continue
                opp = [j for j, u in enumerate(st) if j != i and not u[4] and dirs[j] != dirs[i]
                       and math.hypot(u[0] - s_[0], u[1] - s_[1]) <= DETECT_RANGE]
                if len(opp) >= 2: return 1
        return 0
    T = list(itertools.product([2.8, 3.2, 3.6], [3, 4], ["unloaded", "mixed", "loaded"], ["DLTO", "ORCA", "FLC_ORCA"], range(30)))
    with Pool(2) as pl: h = pl.map(job, T, chunksize=20)
    print(f"\nRuns in which a vehicle faced >= 2 opposite vehicles within d_detect: {sum(h)} of {len(T)}")
