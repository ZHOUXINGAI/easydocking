# 20260415 latest batch (M1-M4, round5 close-front guard)

Command (5x): `START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260415_234355_px4_sih`
- `20260415_234745_px4_sih`
- `20260415_235031_px4_sih`
- `20260415_235311_px4_sih`
- `20260415_235703_px4_sih`

## Headline

- final-pass: `3/5`
- M1 same-direction: `3/5`
- M2 docking-front strict: `1/5`
- M3 short-endgame: `3/5`
- M4 final-pass: `3/5`
- all M1-M4: `0/5`

## Comparison

- baseline 12:10: final-pass `3/5`
- previous kept round4: final-pass `4/5`, M2 `1/5`, all `1/5`
- historical fix round: final-pass `4/5`, M2 `2/5`
- round5: final-pass `3/5`, M2 `1/5`, all `0/5`

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | straight_sec | dock_to_completed_sec | m1_same_direction_ok | m2_front_ok | m3_short_endgame_ok | m4_final_pass_ok | all_m1_m4_ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260415_234355_px4_sih | DOCKING | 0.000000 | 73.100000 | 81.300000 | - | 0.973820 | -0.857506 | 0.971646 | -0.857506 | 3.360000 | 0.000000 | 118.440000 | - | True | False | False | False | False |
| 20260415_234745_px4_sih | COMPLETED | 1.000000 | 82.180000 | 107.480000 | 113.540000 | 0.963317 | -0.253754 | 0.809917 | -0.253754 | 0.920000 | 0.000000 | 5.880000 | 6.060000 | True | False | True | True | False |
| 20260415_235031_px4_sih | COMPLETED | 1.000000 | 87.040000 | 96.740000 | 101.640000 | 0.873288 | -0.721609 | 0.826531 | -0.283897 | 0.850000 | 0.041322 | 6.150000 | 4.900000 | False | False | True | True | False |
| 20260415_235311_px4_sih | DOCKING | 0.000000 | 82.700000 | 93.100000 | - | 0.994350 | -0.244593 | 1.000000 | 1.761197 | 0.000000 | 0.074380 | 102.601000 | - | False | True | False | False | False |
| 20260415_235703_px4_sih | COMPLETED | 1.000000 | 82.240000 | 91.740000 | 98.480000 | 0.886154 | -1.142053 | 0.725926 | -1.142053 | 1.850000 | 0.000000 | 7.950000 | 6.740000 | True | False | True | True | False |

Raw metrics CSV: `report/20260415_batch5_m1m4_latest_r5_metrics.csv`
