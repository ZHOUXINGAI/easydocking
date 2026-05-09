# 20260415 latest batch (M1-M4)

Command (5x): `START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260415_195001_px4_sih`
- `20260415_195231_px4_sih`
- `20260415_195502_px4_sih`
- `20260415_195855_px4_sih`
- `20260415_200136_px4_sih`

## Headline

- final-pass: `3/5`
- M1 same-direction: `4/5`
- M2 docking-front strict: `0/5`
- M3 short-endgame: `2/5`
- M4 final-pass: `3/5`
- all M1-M4: `0/5`

## Comparison

- baseline (12:10 batch) final-pass: `3/5`
- previous fix round final-pass: `4/5`
- previous fix round all M1-M4: `1/5`
- latest final-pass: `3/5`
- latest all M1-M4: `0/5`
- latest M1/M2/M3/M4: `4/5`, `0/5`, `2/5`, `3/5`

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | straight_sec | dock_to_completed_sec | m1_same_direction_ok | m2_front_ok | m3_short_endgame_ok | m4_final_pass_ok | all_m1_m4_ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260415_195001_px4_sih | COMPLETED | 1.000000 | 72.840000 | 85.240000 | 93.740000 | 0.875598 | -0.745421 | 0.694118 | -0.745421 | 3.120000 | 0.000000 | 11.700000 | 8.500000 | True | False | False | True | False |
| 20260415_195231_px4_sih | DOCKING | 0.000000 | 82.640000 | 90.540000 | - | 0.766551 | -0.993913 | 0.480620 | -0.993913 | 3.350000 | 0.000000 | 6.400000 | - | True | False | False | False | False |
| 20260415_195502_px4_sih | TRACKING | 0.000000 | 83.400000 | - | - | 1.000000 | 1.428147 | - | - | - | 0.000000 | 0.000000 | - | True | False | False | False | False |
| 20260415_195855_px4_sih | COMPLETED | 1.000000 | 86.480000 | 96.340000 | 103.580000 | 0.956140 | -0.602633 | 0.880952 | -0.602633 | 0.600000 | 0.000000 | 6.800000 | 7.240000 | True | False | True | True | False |
| 20260415_200136_px4_sih | COMPLETED | 1.000000 | 86.940000 | 95.380000 | 103.180000 | 0.867692 | -0.569403 | 0.724359 | -0.569403 | 1.720000 | 0.057851 | 7.200000 | 7.800000 | False | False | True | True | False |

Raw metrics CSV: `report/20260415_batch5_m1m4_latest_metrics.csv`
