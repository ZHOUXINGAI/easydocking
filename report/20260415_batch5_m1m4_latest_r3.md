# 20260415 latest batch (M1-M4, round3)

Command (5x): `START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260415_213021_px4_sih`
- `20260415_213410_px4_sih`
- `20260415_213647_px4_sih`
- `20260415_214038_px4_sih`
- `20260415_214325_px4_sih`

## Headline

- final-pass: `3/5`
- M1 same-direction: `5/5`
- M2 docking-front strict: `2/5`
- M3 short-endgame: `3/5`
- M4 final-pass: `3/5`
- all M1-M4: `1/5`

## Comparison

- baseline (12:10): final-pass `3/5`
- fix round: final-pass `4/5`, all `1/5`
- round1: final-pass `3/5`, all `0/5`
- round2: final-pass `4/5`, all `0/5`
- round3: final-pass `3/5`, all `1/5`

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | straight_sec | dock_to_completed_sec | m1_same_direction_ok | m2_front_ok | m3_short_endgame_ok | m4_final_pass_ok | all_m1_m4_ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260415_213021_px4_sih | DOCKING | 0.000000 | 87.140000 | 96.640000 | - | 0.982239 | -1.264208 | 0.991495 | -1.179770 | 0.840000 | 0.000000 | 100.080000 | - | True | False | False | False | False |
| 20260415_213410_px4_sih | COMPLETED | 1.000000 | 87.340000 | 95.640000 | 99.780000 | 0.915663 | -0.653059 | 0.746988 | -0.653059 | 1.050000 | 0.000000 | 5.450000 | 4.140000 | True | False | True | True | False |
| 20260415_213647_px4_sih | DOCKING | 0.000000 | 82.600000 | 107.600000 | - | 1.000000 | 0.138825 | 1.000000 | 0.138825 | 0.000000 | 0.000000 | 86.940000 | - | True | True | False | False | False |
| 20260415_214038_px4_sih | COMPLETED | 1.000000 | 83.780000 | 110.780000 | 115.580000 | 0.987421 | -0.099838 | 0.916667 | -0.099838 | 0.400000 | 0.000000 | 6.050000 | 4.800000 | True | True | True | True | True |
| 20260415_214325_px4_sih | COMPLETED | 1.000000 | 82.280000 | 90.380000 | 96.640000 | 0.804878 | -0.685896 | 0.552000 | -0.685896 | 2.240000 | 0.000000 | 6.040000 | 6.260000 | True | False | True | True | False |

Raw metrics CSV: `report/20260415_batch5_m1m4_latest_r3_metrics.csv`
