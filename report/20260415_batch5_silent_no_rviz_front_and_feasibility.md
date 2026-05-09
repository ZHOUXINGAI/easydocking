# Silent (No RViz window) Batch 5: front-consistency + feasibility

Command: `START_RVIZ=false AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- final-pass: `1/5`
- TRACKING+DOCKING ahead ratio mean: `0.972` (worst min `-0.740 m`)
- DOCKING ahead ratio mean: `0.969` (worst min `-0.740 m`, mean negative duration `0.175 s`)
- first-6s opposite-direction ratio mean: `0.020`

## Sim-to-real envelope

- Carrier command speed p95 mean: `14.43 m/s`
- Carrier command acceleration p95 mean: `2.50 m/s^2` (now norm-limited, target <= `2.5 m/s^2`)
- Carrier actual speed p95 mean: `12.57 m/s`
- Carrier actual acceleration p95 mean: `7.78 m/s^2`
- Mini TAS setpoint p95 mean: `10.83 m/s`; measured TAS p95 mean: `12.06 m/s`

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | carrier_cmd_acc_p95_mps2 | mini_tas_p95_mps |
|---|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260415_103517_px4_sih | COMPLETED | 1 | 68.02 | 82.56 | 89.12 | 0.927 | -0.740 | 0.885 | -0.740 | 0.700 | 2.499 | 12.550 |
| 20260415_103746_px4_sih | DOCKING | 0 | 82.60 | 90.50 | - | 0.936 | -0.312 | 1.000 | 3.971 | 0.000 | 2.500 | 11.964 |
| 20260415_104019_px4_sih | DOCKING | 0 | 84.30 | 92.04 | - | 1.000 | 0.069 | 1.000 | 2.689 | 0.000 | 2.499 | 11.990 |
| 20260415_104256_px4_sih | DOCKING | 0 | 82.38 | 90.48 | - | 0.996 | -0.091 | 0.989 | -0.091 | 0.000 | 2.500 | 11.919 |
| 20260415_104533_px4_sih | TRACKING | 0 | 87.44 | - | - | 1.000 | 0.244 | nan | nan | 0.000 | 2.500 | 11.867 |
