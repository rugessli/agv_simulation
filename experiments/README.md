# Supplementary experiments (paper Sections 5.4, 6.8 and 6.9)

These scripts use the simulation code in the repository root **unchanged**; they only
override the priority weights at run time (ablation) or wrap the planning step with a
model of imperfect information (robustness). Run them from this folder.

| Script | Experiment | Runs | Output |
|---|---|---|---|
| `run_ablation.py` | DLTO without the cargo-state term (alpha = 0) and without any cargo information (alpha = beta = 0), plus ORCA reference | 3,240 | `results/ablation.csv` |
| `run_robust.py` | DLTO, ORCA and FLC-ORCA under 7 conditions of report period, latency, message loss and localisation error | 17,010 | `results/robustness.csv` |
| `DISCARD_AFTER_TIMEOUT=1 python run_robust.py ...` | Variant in which a vehicle whose reports stop is discarded after 2 s instead of being dead-reckoned | 17,010 | `results/robustness_discard_after_timeout.csv` |
| `analyze_ablation.py`, `analyze_robust.py` | Summaries reported in Tables 8 and 9 | – | stdout |

```
python run_ablation.py
python run_robust.py C0_ideal,C1_0.5s,C2_1.0s,C3_loss30,C4_loc2cm,C5_loc5cm,C6_worst 30 results/robustness.csv
python analyze_ablation.py
python analyze_robust.py
```

Robustness model (see the docstring of `run_robust.py`): each AGV plans with its own pose
estimate (true pose plus a first-order autoregressive localisation error, correlation time 2 s)
and with the latest pose reports of the other vehicles (period T_rep, latency t_lat,
independent loss p_loss per receiver, last report propagated at its reported velocity).
Within d_detect = 3 m the other vehicle's pose comes from the laser scanner (relative to the
receiver). Encounters are detected and clearances measured on the true poses.

Key results: removing all cargo information reduces the mixed-payload MCD gain over ORCA
from +5.5 % to +3.0 % and the unloaded vehicle no longer reliably provides the extra clearance;
report periods up to 1.0 s, latency up to 0.2 s and 30 % message loss leave MCD unchanged;
a localisation error of 50 mm (5x the platform specification) brings 18 % of the 2.8 m
corridor runs below d_min for all three algorithms, with no contact; the DLTO gain over ORCA
stays between +5.4 % and +6.8 % in all conditions.
