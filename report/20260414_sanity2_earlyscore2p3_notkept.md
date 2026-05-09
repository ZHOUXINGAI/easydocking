# Sanity Check (Early release score threshold 2.3, not kept)

Command: `... AUTO_START_REAR_ENTRY_ENERGY_EARLY_RELEASE_SCORE_THRESHOLD=2.3 ...` (2x)

- final-pass: `1/2`
- docking ahead ratio mean `0.482`, docking ahead min mean `-1.599`
- path mean `126.06m`
- decision: **not kept** (stability failed in sanity: 1/2).

| run_id | final_pass | first_tracking_t_sec | first_docking_t_sec | first_completed_t_sec | ahead_ratio_positive_pre_completed | ahead_min_m | docking_ahead_ratio_positive | docking_ahead_min_m | docking_lateral_abs_max_m | carrier_path_until_completed_m |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 20260414_230210_px4_sih | 0 | 82.00 | 91.10 | nan | 0.837 | -1.961 | 0.354 | -1.961 | 0.794 | 124.94 |
| 20260414_230457_px4_sih | 1 | 82.38 | 91.04 | 97.84 | 0.878 | -1.237 | 0.610 | -1.237 | 1.441 | 127.18 |
