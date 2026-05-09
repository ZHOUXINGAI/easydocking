# Batch Check (User-Issue Fix: Early Docking + Reduced End Wobble) — r6

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh` (5x)

- Changes in this round: keep ahead hard-constraint strengthened (`8.7/1.10/5.6`) + allow early TRACKING even with large vertical gap + close-tracking release supports APPROACH + tangent minimum hold `3.6s` + tightened terminal lateral smoothing/rate-limit.
- final-pass: `5/5`
- first TRACKING time: mean `85.74s` (range `83.58s ~ 87.28s`)
- first DOCKING time: mean `94.52s` (best `91.68s`)
- first COMPLETED time: mean `99.48s` (best `97.34s`)
- tangent start time: mean `90.41s` (range `86.98s ~ 91.98s`)
- end wobble (DOCKING lateral): mean max |lat| `0.92m`, worst `1.53m`, mean sign flips `2.2`
- ahead metric: mean ratio `0.904`, mean min `-1.228` (worst `-1.579`)
- approach orbit gap mean: `6.78m`

| run_id | final_pass | first_tracking_t_sec | tangent_start_t_sec | first_docking_t_sec | first_completed_t_sec | docking_lateral_abs_max_m | ahead_min_m |
|---|---:|---:|---:|---:|---:|---:|---:|
| 20260412_205513_px4_sih | 1 | 86.14 | 90.98 | 94.94 | 99.64 | 1.106 | -1.537 |
| 20260412_205753_px4_sih | 1 | 83.58 | 86.98 | 91.68 | 97.34 | 0.918 | -0.661 |
| 20260412_210030_px4_sih | 1 | 87.28 | 91.48 | 95.94 | 99.84 | 0.488 | -0.961 |
| 20260412_210310_px4_sih | 1 | 85.04 | 90.64 | 94.24 | 98.64 | 1.527 | -1.403 |
| 20260412_210551_px4_sih | 1 | 86.68 | 91.98 | 95.78 | 101.94 | 0.562 | -1.579 |
