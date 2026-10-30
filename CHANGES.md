# Changes in this commit

## 1. Hold-rule fix (applies equally to DLTO, ORCA and FLC-ORCA)
- `config.py`: added `WAIT_OFFSET_TOL = 0.05` and `hold_distance(partner_err)`.
- `simulation.py`, `orca_algorithm.py`, `flc_orca_algorithm.py`: the wait-for-offset (hold) rule
  - releases only when the partner's lateral error is <= 0.05 m (previously 0.25 m), and
  - is triggered within `hold_distance = max(DETECT_RANGE, AGV_LENGTH + 2*sqrt(AGV_MIN_TURN_R*e_partner) + 0.5)`
    instead of the fixed `DETECT_RANGE` (3 m).

Reason: with 4 AGVs in the 2.8 m corridor, passing started while a vehicle was still completing its
offset with a tilted body; the measured MCD fell below D_MIN (0.25 m) in ~90 runs per algorithm
(minimum 0.056 m). After the fix no run falls below D_MIN (minimum 0.305 m), DRR stays 1.000,
and the obstacle experiment is unchanged.

## 2. Reproducibility of the paper
- `sensitivity_full.py` (new): sensitivity analysis as reported in the paper
  (alpha 0.30-0.70, beta:gamma:epsilon kept at 5:4:1, full factorial, 810 DLTO runs per alpha).
- `figures/`: scripts that generate Figs. 2-4 of the paper.
- `output/`: all results regenerated with this code
  (`results.csv`, `results_summary.csv`, `results_obstacle.csv`, `metric_comparison.png`,
  `sensitivity_results.csv`, `sensitivity_full.csv`, `paper_figures/`).

## Reproduce
    python main.py                        # 3,240 main runs  -> output/results.csv
    python run_obstacles.py               # 1,080 obstacle runs -> output/results_obstacle.csv
    python sensitivity_full.py            # paper sensitivity table -> output/sensitivity_full.csv
    python figures/fig4_main_results.py   # Fig. 4
    python figures/fig3_snapshots.py      # Fig. 3
    python figures/fig2_state_machine.py  # Fig. 2
