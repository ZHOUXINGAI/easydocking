# RViz-silent Batch (5 runs): front-consistency + sim-to-real envelope

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `2/5`
- TRACKING+DOCKING ahead ratio mean: `0.975` (worst min `-1.037 m`)
- DOCKING ahead ratio mean: `0.898` (worst min `-1.037 m`, mean negative duration `0.385 s`)
- first-6s opposite-direction ratio (`opp_ratio_6s_speedg1`) stays low in all runs

## Sim-to-real envelope (from this batch)

- Carrier command speed p95 mean: `14.04 m/s` (limit configured `14.5 m/s`)
- Carrier command accel p95 mean: `3.40 m/s^2` (configured accel limit parameter `2.5 m/s^2`, but per-axis clamp allows norm up to ~`3.54 m/s^2`)
- Carrier actual speed p95 mean: `10.31 m/s`
- Carrier actual accel p95 mean: `5.60 m/s^2` (contains estimator/noise spikes)
- Mini TAS setpoint p95 mean: `10.56 m/s`; measured TAS p95 mean: `11.55 m/s`

Interpretation:
- Fixed-wing side is mostly in a plausible cruise band (~9–12 m/s TAS target, p95 around ~11–12 m/s).
- Carrier side is aggressive for a generic heavy quad platform; feasible for high-performance multirotors, risky for larger payload carriers.
- For stronger sim-to-real, enforce acceleration by vector norm (not per-axis) and add low-pass/jerk limit before publishing offboard velocity setpoints.

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | carrier_cmd_speed_p95_mps | carrier_cmd_acc_p95_mps2 | mini_tas_p95_mps |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260415_002326_px4_sih | DOCKING | 0 | 83.46 | 91.50 | - | 0.999 | -0.016 | 1.000 | 1.742 | 0.000 | 13.550 | 3.327 | 11.430 |
| 20260415_002732_px4_sih | COMPLETED | 1 | 83.30 | 92.04 | 96.20 | 0.934 | -1.037 | 0.795 | -1.037 | 0.800 | 14.491 | 3.536 | 11.968 |
| 20260415_003018_px4_sih | DOCKING | 0 | 85.70 | 95.10 | - | 1.000 | 0.226 | 1.000 | 1.812 | 0.000 | 14.374 | 3.063 | 11.309 |
| 20260415_003415_px4_sih | COMPLETED | 1 | 83.64 | 93.74 | 97.70 | 0.943 | -0.544 | 0.797 | -0.544 | 0.740 | 14.484 | 3.536 | 11.919 |
| 20260415_003652_px4_sih | TRACKING | 0 | 83.14 | - | - | 1.000 | 1.742 | nan | nan | 0.000 | 13.298 | 3.536 | 11.126 |
