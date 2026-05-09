# 20260415 latest batch (M1-M4, round4)

Command (5x): `START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run set:
- `20260415_215418_px4_sih`
- `20260415_215652_px4_sih`
- `20260415_215939_px4_sih`
- `20260415_220214_px4_sih`
- `20260415_220440_px4_sih`

## Headline

- final-pass: `4/5`
- M1 same-direction: `4/5`
- M2 docking-front strict: `1/5`
- M3 short-endgame: `3/5`
- M4 final-pass: `4/5`
- all M1-M4: `1/5`

## Comparison

- baseline 12:10: final-pass `3/5`
- fix round: final-pass `4/5`, all `1/5`
- round1: final-pass `3/5`, M2 `0/5`, all `0/5`
- round2: final-pass `4/5`, M2 `1/5`, all `0/5`
- round3: final-pass `3/5`, M2 `2/5`, all `1/5`
- round4: final-pass `4/5`, M2 `1/5`, all `1/5`

## Per-run

| run_id | phase_end | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | track_ahead_ratio_positive | track_ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_negative_duration_sec | opp_ratio_6s_speedg1 | straight_sec | dock_to_completed_sec | m1_same_direction_ok | m2_front_ok | m3_short_endgame_ok | m4_final_pass_ok | all_m1_m4_ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 20260415_215418_px4_sih | COMPLETED | 1.000000 | 87.340000 | 95.540000 | 99.240000 | 0.941176 | -0.868635 | 0.810811 | -0.868635 | 0.700000 | 0.000000 | 4.950000 | 3.700000 | True | False | True | True | False |
| 20260415_215652_px4_sih | COMPLETED | 1.000000 | 83.240000 | 110.700000 | 115.540000 | 1.000000 | 1.624422 | 1.000000 | 1.624422 | 0.000000 | 0.000000 | 6.050000 | 4.840000 | True | True | True | True | True |
| 20260415_215939_px4_sih | COMPLETED | 1.000000 | 86.800000 | 95.300000 | 99.240000 | 0.943775 | -0.743439 | 0.822785 | -0.743439 | 0.700000 | 0.000000 | 5.150000 | 3.940000 | True | False | True | True | False |
| 20260415_220214_px4_sih | COMPLETED | 1.000000 | 68.080000 | 83.640000 | 92.740000 | 0.953347 | -0.448405 | 0.873626 | -0.448405 | 0.920000 | 0.000000 | 8.320000 | 9.100000 | True | False | False | True | False |
| 20260415_220440_px4_sih | APPROACH | 0.000000 | - | - | - | - | - | - | - | - | - | 0.000000 | - | False | False | False | False | False |

Raw metrics CSV: `report/20260415_batch5_m1m4_latest_r4_metrics.csv`
