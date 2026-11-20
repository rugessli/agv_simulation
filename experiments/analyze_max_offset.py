"""Summarise results/max_offset.csv."""
import pandas as pd
d = pd.read_csv("results/max_offset.csv")
print(d.groupby("algorithm")[["DRR", "ART", "ST", "MCD"]].mean().round(4).to_string())
for w in (3.2, 3.6):
    x = d[(d.corridor_width == w) & (d.payload_profile != "unloaded")]
    print(f"\nw = {w} m (runs with loaded vehicles)")
    print(x.groupby("algorithm").agg(MCD=("MCD", "mean"), wall_loaded_mean=("wall_loaded", "mean"),
          wall_loaded_min=("wall_loaded", "min"), maxy_loaded=("maxy_loaded", "mean")).round(3).to_string())
