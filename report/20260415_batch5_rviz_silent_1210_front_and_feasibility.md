# RViz-silent Batch (5 runs, 12:10 window): front-consistency + sim-to-real envelope

Command (5x):
`START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260415_121033_px4_sih`
- `20260415_121428_px4_sih`
- `20260415_121703_px4_sih`
- `20260415_121928_px4_sih`
- `20260415_122202_px4_sih`

## Batch headline

- final-pass: `3/5`
- TRACKING+DOCKING ahead ratio mean: `0.915` (worst min `-0.926 m`)
- DOCKING ahead ratio mean: `0.650` (worst min `-0.926 m`, mean negative duration `2.197 s`, max `3.420 s`)
- first-6s opposite-direction ratio (`opp_ratio_6s_speedg1`) mean: `0.010`

Interpretation:
- Early opposite-direction behavior is mostly controlled (first 6s almost always non-opposite).
- Main shortboard remains DOCKING stage front-consistency: mini still overtakes carrier in 3 completed runs for non-trivial windows.
- Two runs stayed in TRACKING (`121033`, `122202`) and never entered DOCKING.

## Sim-to-real envelope (this batch)

- Carrier command speed p95 mean: `14.15 m/s` (close to configured limit `14.5 m/s`)
- Carrier command accel p95 mean: `3.52 m/s^2` (consistent with XY per-axis clamp norm ceiling around `3.54 m/s^2`)
- Carrier actual speed p95 mean: `11.44 m/s`
- Carrier actual accel p95 mean: `6.94 m/s^2` (contains estimator/noise spikes)
- Mini TAS setpoint p95 mean: `10.35 m/s`; measured TAS p95 mean: `11.80 m/s`

Interpretation:
- Fixed-wing side is still in realistic cruise band for small platforms.
- Carrier side remains aggressive for heavier multirotor payload scenarios.
- If we want stronger sim-to-real transfer, next should be runtime/window tuning first (reduce long TRACKING tails), then controller-level smoothness constraints.

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | carrier_cmd_speed_p95_mps | carrier_cmd_acc_p95_mps2 | mini_tas_p95_mps |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260415_121033_px4_sih | TRACKING | 0 | 83.14 | - | - | 1.000 | 1.137 | nan | nan | nan | 0.000 | 14.119 | 3.481 | 11.162 |
| 20260415_121428_px4_sih | COMPLETED | 1 | 71.72 | 81.96 | 88.36 | 0.826 | -0.676 | 0.547 | -0.676 | 2.320 | 0.000 | 14.477 | 3.536 | 12.558 |
| 20260415_121703_px4_sih | COMPLETED | 1 | 73.34 | 81.70 | 89.10 | 0.819 | -0.713 | 0.615 | -0.713 | 3.420 | 0.000 | 14.488 | 3.536 | 12.368 |
| 20260415_121928_px4_sih | COMPLETED | 1 | 83.18 | 91.14 | 95.14 | 0.929 | -0.926 | 0.787 | -0.926 | 0.850 | 0.050 | 14.489 | 3.536 | 11.909 |
| 20260415_122202_px4_sih | TRACKING | 0 | 67.88 | - | - | 1.000 | 0.854 | nan | nan | nan | 0.000 | 13.181 | 3.536 | 10.984 |

Raw metrics CSV: `report/20260415_batch5_rviz_silent_1210_front_and_feasibility_metrics.csv`
