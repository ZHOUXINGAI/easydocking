# Silent (No RViz) batch5 after front-guard patch (valid runs)

Command: `START_RVIZ=false AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `0/5`
- TRACKING+DOCKING ahead ratio mean: `1.000` (worst min `0.535 m`)
- DOCKING ahead ratio mean: `1.000` (worst min `1.089 m`, mean negative duration `0.000 s`)
- first-6s opposite-direction ratio mean: `0.000`

## Feasibility snapshot

- Carrier command accel p95 mean: `3.54 m/s^2`
- Carrier actual speed p95 mean: `11.82 m/s`
- Mini TAS setpoint p95 mean: `10.83 m/s`; measured TAS p95 mean: `12.14 m/s`

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | carrier_cmd_acc_p95_mps2 |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260415_111500_px4_sih | TRACKING | 0 | 74.50 | - | - | 1.000 | 0.710 | nan | nan | 0.000 | 3.536 |
| 20260415_111733_px4_sih | TRACKING | 0 | 66.26 | - | - | 1.000 | 1.043 | nan | nan | 0.000 | 3.536 |
| 20260415_112005_px4_sih | TRACKING | 0 | 73.10 | - | - | 1.000 | 0.787 | nan | nan | 0.000 | 3.536 |
| 20260415_112237_px4_sih | TRACKING | 0 | 87.10 | - | - | 1.000 | 0.535 | nan | nan | 0.000 | 3.536 |
| 20260415_112512_px4_sih | TRACKING | 0 | 68.14 | 76.34 | - | 1.000 | 0.535 | 1.000 | 1.089 | 0.000 | 3.536 |
