# Runtime tuning sanity (not kept)

Goal: improve `TRACKING -> DOCKING` conversion and reduce DOCKING front-loss without introducing new controller-side instability.

## Baseline reference (latest 5-run RViz-silent batch)

- Reference report: `report/20260415_batch5_rviz_silent_1210_front_and_feasibility.md`
- Baseline final-pass: `3/5`
- Baseline DOCKING ahead ratio mean: `0.650` (completed runs have non-trivial back-cross windows)

## Sanity trials (2 runs each)

### A) `speed14p8_rviz_silent`
Command core: `CARRIER_TRACKING_SPEED_LIMIT=14.8`
- runs: `20260415_131444_px4_sih`, `20260415_131836_px4_sih`
- final-pass: `1/2`
- DOCKING ahead ratio mean: `0.634`
- verdict: no stable improvement (still TRACKING-only failure + docking front-loss)

### B) `mini_slow_docking_capture`
Command core: `MINI_DOCKING_SPEED_COMMAND=7.0 MINI_CAPTURE_SPEED_COMMAND=5.2`
- runs: `20260415_132328_px4_sih`, `20260415_132541_px4_sih`
- final-pass: `1/2`
- DOCKING ahead ratio mean: `0.800`
- `opp_ratio_6s_speedg1` worsened in one run (`0.074`)
- verdict: not promoted

### C) `first_entry_max_ahead_6p6_reverted`
Code experiment: first-entry max ahead distance `6.0 -> 6.6` (then reverted)
- runs: `20260415_134032_px4_sih`, `20260415_134313_px4_sih`
- final-pass: `1/2`
- DOCKING ahead ratio mean: `0.825`
- verdict: evidence insufficient and unstable; code change reverted

### D) `early_release_score_1p9`
Command core: `AUTO_START_REAR_ENTRY_ENERGY_EARLY_RELEASE_SCORE_THRESHOLD=1.9`
- runs: `20260415_134835_px4_sih`, `20260415_135224_px4_sih`
- final-pass: `1/2` (one run stuck in DOCKING, one run completed with front-loss)
- DOCKING ahead ratio mean: `0.826`
- verdict: not promoted

## Conclusion

- None of the above runtime sanity tweaks beats the current baseline in a stable way.
- Current promoted state remains: baseline controller behavior + `RVIZ_SILENT` execution support.
- Next step should be diagnostic-first (gating-level failure attribution around `TRACKING->DOCKING` and DOCKING hold criteria), then targeted parameterization of entry gates.

Raw metrics CSV: `report/20260415_runtime_tuning_sanity_notkept_metrics.csv`
