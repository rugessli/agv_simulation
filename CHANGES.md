# Changes on branch `jirs-revision`

## 1. FLC-ORCA closing speed (bug fix)
- `flc_orca_algorithm.py`: in a head-on encounter the two vehicles move towards each other, so the
  closing speed is now `a.v + partner.v` (previously the difference of the speeds, which is close
  to zero when both drive at the same speed). Docstring now credits Mohd Romlay et al. (2023) and
  states that no velocity-obstacle half-planes are computed.
- Effect: only the FLC-ORCA rows of `output/results.csv` change (DLTO, ORCA and SW are identical
  to `main`). FLC-ORCA overall MCD 0.5047 -> 0.5136 m; DLTO vs FLC-ORCA in mixed payloads
  +4.6 % -> +2.7 % (p < 0.001); under all-loaded payloads FLC-ORCA is now 1.9 % above DLTO.
  The obstacle experiment is unchanged.

## 2. Statistics script
- `statistics_main.py` (new): tables of DRR/ART/ST/NEC/MCD, paired Wilcoxon tests with Cohen's dz
  and the fleet-size effect, as reported in Section 6. `--encounters` also counts the 3-4 AGV runs
  in which a vehicle faced two or more oncoming vehicles within d_detect (28 of 1,620).
  Output: `output/statistics_main.txt`.

## 3. Deadlock confirmation time
- `config.py`: `T_DEAD = 3.0` to match Section 3.2 (was 2.0). The constant is not used by the
  metrics, so no result changes.

## 4. Maximum-offset reference baseline
- `orca_algorithm.py`: `orca_step(..., responsibility=0.5)`; new `max_offset_step` uses
  responsibility 1.0 (both vehicles at delta_max). Docstring corrected (no half-planes).
- `simulation.py`: algorithm name `"MAX_OFFSET"`.
- `experiments/run_max_offset.py`, `experiments/analyze_max_offset.py` (new): runs DLTO, ORCA,
  FLC-ORCA and MAX_OFFSET over the full factorial design and records the minimum wall clearance
  of loaded vehicles -> `experiments/results/max_offset.csv` (paper Section 6.10, Table 10).

## 5. Re-run of all affected experiments with this code
- `output/results.csv`, `results_summary.csv`, `results_obstacle.csv`, `paper_figures/fig3, fig4`.
- `experiments/results/robustness.csv` (17,010 runs; DLTO and ORCA identical to `main`).
- `experiments/results/robustness_discard_after_timeout.csv`: now produced with the current
  robustness wrapper for the two conditions reported in the paper (C3, C6; 4,860 runs). The file
  on `main` came from an earlier wrapper version; with the current code, 6 of 810 runs per algorithm
  (0.7 %) end unresolved with contact under 30 % message loss (paper previously: 9 runs, 1.1 %).
- Ablation and sensitivity results do not involve FLC-ORCA and are unchanged.

---

# Earlier changes (commit e198419)

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
