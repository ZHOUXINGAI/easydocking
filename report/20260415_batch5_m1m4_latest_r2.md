# 20260415 latest batch (M1-M4, round2)

Command (5x): `START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260415_210750_px4_sih`
- `20260415_211012_px4_sih`
- `20260415_211405_px4_sih`
- `20260415_211639_px4_sih`
- `20260415_211905_px4_sih`

## Headline

- final-pass: `4/5`
- M1 same-direction: `5/5`
- M2 docking-front strict: `1/5`
- M3 short-endgame: `3/5`
- M4 final-pass: `4/5`
- all M1-M4: `0/5`

## Comparison

- baseline (12:10 batch): final-pass `3/5`
- previous fix round: final-pass `4/5`, all M1-M4 `1/5`
- previous latest round1: final-pass `3/5`, all M1-M4 `0/5`
- current round2: final-pass `4/5`, all M1-M4 `0/5`

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | straight_sec | dock_to_completed_sec | m1_same_direction_ok | m2_front_ok | m3_short_endgame_ok | m4_final_pass_ok | all_m1_m4_ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260415_210750_px4_sih | COMPLETED | 1.000000 | 83.240000 | 92.740000 | 96.740000 | 0.937037 | -0.875237 | 0.787500 | -0.875237 | 1.020000 | 0.000000 | 6.300000 | 4.000000 | True | False | True | True | False |
| 20260415_211012_px4_sih | DOCKING | 0.000000 | 82.980000 | 131.140000 | - | 0.953632 | -1.969629 | 1.000000 | 0.936680 | 0.000000 | 0.000000 | 39.120000 | - | True | True | False | False | False |
| 20260415_211405_px4_sih | COMPLETED | 1.000000 | 83.240000 | 92.840000 | 98.440000 | 0.865132 | -1.412249 | 0.633929 | -1.412249 | 1.640000 | 0.000000 | 5.520000 | 5.600000 | True | False | True | True | False |
| 20260415_211639_px4_sih | COMPLETED | 1.000000 | 73.240000 | 85.080000 | 92.780000 | 0.836317 | -0.926850 | 0.584416 | -0.926850 | 3.200000 | 0.000000 | 8.900000 | 7.700000 | True | False | False | True | False |
| 20260415_211905_px4_sih | COMPLETED | 1.000000 | 73.140000 | 83.040000 | 86.800000 | 0.941392 | -0.976824 | 0.786667 | -0.976824 | 0.800000 | 0.000000 | 5.050000 | 3.760000 | True | False | True | True | False |

Raw metrics CSV: `report/20260415_batch5_m1m4_latest_r2_metrics.csv`
