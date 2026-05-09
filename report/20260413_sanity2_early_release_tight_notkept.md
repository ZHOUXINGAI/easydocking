# Sanity Check (early-release defaults tightened, not kept)

Command: `START_RVIZ=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh (2x)`

- change tried: `AUTO_START_REAR_ENTRY_ENERGY_EARLY_RELEASE_MIN_AFTER_ORBIT_SEC 12.0->16.0`, `HOLD_COUNT 4->5`, `SCORE_THRESHOLD 1.9->1.7`
- final-pass: `1/2`
- docking ahead ratio mean `0.657`, docking ahead min worst `-2.740`
- docking lateral max |error| mean `0.784m`, carrier path mean `150.15m`

- conclusion: robustness regressed (`1/2`), so defaults were reverted.

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260413_145856_px4_sih | 1 | 83.26 | 91.54 | 101.50 | 0.925 | -1.427 | 0.805 | -1.427 | 0.712 | 153.06 |
| 20260413_150139_px4_sih | 0 | 83.84 | 93.34 | nan | 0.849 | -2.740 | 0.510 | -2.740 | 0.856 | 147.24 |
