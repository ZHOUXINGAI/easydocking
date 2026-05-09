# Batch Check (Early Tangent + Lateral Smoothing) — r5

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- Changes in this round: close-tracking gate expanded to allow APPROACH release + tangent exit min hold reduced to `3.6s` + stronger DOCKING lateral smoothing/rate-limit.
- final-pass: `5/5`
- start timing (`first_non_idle_t_sec`): mean `76.36s` (range `75.38s ~ 79.40s`)
- mini tangent start (`tangent_start_t_sec`): mean `86.04s` (range `85.04s ~ 89.04s`)
- first docking entry (`first_docking_t_sec`): mean `95.93s`
- phase duration mean: `APPROACH 18.50s`, `TRACKING 1.08s`, `DOCKING 5.23s`
- docking lateral smoothness: mean sign flips `1.0`, mean max |lat| `0.54m`
- ahead metric: mean `ahead_ratio_positive_pre_completed = 0.889`, mean `ahead_min_m = -2.049` (worst `-2.638`)
- approach carrier-orbit gap mean: `9.97m`

| run_id | final_pass | first_non_idle_t_sec | tangent_start_t_sec | first_docking_t_sec | first_completed_t_sec | approach_sec | tracking_sec | docking_sec | ahead_ratio_positive_pre_completed | ahead_min_m | approach_carrier_orbit_gap_mean_m | docking_lateral_abs_max_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260412_192134_px4_sih | 1 | 79.40 | 89.04 | 98.44 | 104.74 | 18.14 | 0.90 | 6.26 | 0.880 | -1.825 | 9.73 | 0.473 |
| 20260412_192423_px4_sih | 1 | 75.38 | 85.04 | 94.88 | 99.44 | 18.56 | 0.94 | 4.50 | 0.909 | -2.638 | 10.03 | 0.377 |
| 20260412_192702_px4_sih | 1 | 75.70 | 85.44 | 95.90 | 101.60 | 18.50 | 1.70 | 5.64 | 0.867 | -1.808 | 9.60 | 0.708 |
| 20260412_192944_px4_sih | 1 | 75.86 | 85.60 | 95.50 | 100.96 | 18.68 | 0.96 | 5.40 | 0.878 | -1.545 | 9.61 | 0.653 |
| 20260412_193225_px4_sih | 1 | 75.44 | 85.08 | 94.94 | 99.34 | 18.60 | 0.90 | 4.34 | 0.910 | -2.431 | 10.87 | 0.468 |
