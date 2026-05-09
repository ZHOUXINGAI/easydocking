# Batch Check (LAT gate 1.3 experiment)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh (5x)`

- change: tighten passive tracking->docking lateral gate `1.6 -> 1.3` (experiment only, later reverted)
- final-pass: `5/5`
- first TRACKING time mean `83.37s` (range `81.14~87.54`)
- first DOCKING time mean `91.86s` (range `89.84~96.20`)
- first COMPLETED time mean `99.30s` (range `95.90~103.64`)
- ahead ratio (pre-completed) mean `0.849`, ahead min mean `-1.622` (worst `-2.106`)
- docking ahead ratio mean `0.526`, docking ahead min mean `-1.622` (worst `-2.106`)
- opposite-motion ratio first 6s (carrier speed>1) mean `0.000`
- docking lateral max |error| mean `0.881m` (worst `1.375m`), carrier path until completed mean `135.24m`

- conclusion: this setting reduced average path/lateral peaks, but degraded docking-ahead quality, so it was **not kept**.

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | opp_ratio_6s_speedg1 | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260413_143820_px4_sih | 1 | 83.14 | 91.20 | 97.44 | 0.874 | -2.106 | 0.563 | -2.106 | 0.000 | 0.518 | 120.37 |
| 20260413_144058_px4_sih | 1 | 81.14 | 89.84 | 101.10 | 0.869 | -1.833 | 0.681 | -1.833 | 0.000 | 0.812 | 168.31 |
| 20260413_144340_px4_sih | 1 | 82.30 | 90.36 | 95.90 | 0.873 | -1.483 | 0.518 | -1.483 | 0.000 | 0.541 | 116.40 |
| 20260413_144617_px4_sih | 1 | 82.72 | 91.72 | 98.42 | 0.857 | -1.395 | 0.593 | -1.395 | 0.000 | 1.375 | 130.80 |
| 20260413_144857_px4_sih | 1 | 87.54 | 96.20 | 103.64 | 0.772 | -1.291 | 0.273 | -1.291 | 0.000 | 1.158 | 140.34 |
