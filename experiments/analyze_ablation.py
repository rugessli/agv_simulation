"""Summarise results/ablation.csv (Table 8 of the paper)."""
import pandas as pd, numpy as np
from scipy.stats import wilcoxon
df = pd.read_csv('results/ablation.csv')
key = ['corridor_width', 'n_agv', 'payload_profile', 'seed']
O = df[df.variant == 'ORCA'].set_index(key)
for v in ['DLTO', 'noCargoState', 'noCargoInfo']:
    X = df[df.variant == v].set_index(key); L = X.index.get_level_values; cells = []
    for name, sel in [('mixed', L(2) == 'mixed'), ('mixed 3.2', (L(2) == 'mixed') & (L(0) == 3.2)),
                      ('mixed 3.6', (L(2) == 'mixed') & (L(0) == 3.6)), ('unloaded', L(2) == 'unloaded'), ('loaded', L(2) == 'loaded')]:
        x = X.MCD[sel]; y = O.MCD.loc[x.index]; d = x - y
        cells.append(f"{name}: {100 * (x.mean() / y.mean() - 1):+.1f}% (p={wilcoxon(d[d.abs() > 1e-12]).pvalue:.0e})")
    print(v, ' | '.join(cells))
m = df[(df.payload_profile == 'mixed') & df.maxy_loaded.notna() & (df.corridor_width > 2.9)]
m = m.assign(loaded_more=m.maxy_loaded > m.maxy_unloaded + 0.01, unloaded_more=m.maxy_unloaded > m.maxy_loaded + 0.01)
print(m.groupby('variant')[['loaded_more', 'unloaded_more']].mean().round(3))
print(df[(df.payload_profile == 'mixed') & (df.corridor_width == 3.6)].groupby('variant')[['maxy_loaded', 'maxy_unloaded']].mean().round(3))
