# 2026-04-24 Terminal Smoke `rel_z` Soft-Gate Probe

## Goal
- Test whether accepted-window `rel_z` can be used as a **soft discriminator** to bias `terminal_smoke` away from the long-path family.
- Keep the gate deterministic; avoid another brittle hard threshold.

## Code changes
- `scripts/wait_for_docking_window.py`
  - add tunable soft penalty terms:
    - `rear_entry_prediction_score_min_rel_z_target_m`
    - `rear_entry_prediction_score_min_rel_z_scale_m`
    - `rear_entry_prediction_score_min_rel_z_weight`
  - apply the penalty as an **under-target-only** term in both the local prediction score and the global-intercept geometry score
- `scripts/start_docking_command.sh`
  - plumb the three new parameters into the starter node
- `scripts/run_px4_sih_docking_experiment.sh`
  - plumb the new env vars into metadata / launcher
  - briefly tried `terminal_smoke` defaults `target=3.0`, `weight=1.0`
  - reverted those profile defaults back to neutral (`0.0`, `0.0`) after validation
- `scripts/generate_report.py`
  - make report generation robust to `docking_log.csv` rows containing NUL bytes
- `scripts/classify_px4_sih_result.py`
- `scripts/summarize_px4_sih_batch.py`
  - make batch classification / summarization robust to the same NUL-byte issue

## Data recovered first
- `20260423_212151_px4_sih`
  - `generate_report.py` can now parse it
  - summary:
    - `classification=geometry-fail`
    - `post_start_path_length_m=276.771`
    - `accepted_window_rel_z=2.110`
    - `accepted_window_phase_err_deg=91.7`
    - `accepted_window_pred_score=0.710`

This confirms the previously missing run belongs to the **bad family**, and its accepted `rel_z` is indeed on the low side.

## Validation runs

### 1) Misconfigured carrier baseline — invalid for tuning
- run: `20260424_021329_px4_sih`
- observed metadata:
  - `carrier_activate_on_launch=false`
  - `carrier_idle_hover_altitude=0.4`
- result:
  - `classification=final-pass`
  - `accepted_window_rel_z=31.980`
  - `post_start_path_length_m=292.171`

Interpretation:
- this run is **not representative**
- the carrier was not in the intended prehold state, so the huge accepted `rel_z` is a config artifact, not a useful tuning signal

### 2) Correct carrier baseline + `target=3.0`, `weight=1.0`
- run: `20260424_180725_px4_sih`
- baseline config restored:
  - `CARRIER_ACTIVATE_ON_LAUNCH=true`
  - `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- observed starter behavior before abort:
  - candidate windows stayed around `rel_z ≈ 1.4–2.1`
  - no valid release was reached during the checked interval
  - the new soft term was therefore acting like a practical veto

Interpretation:
- the first tuning was **too aggressive**
- even as a soft score term, `target=3.0`, `weight=1.0` effectively over-constrained the real baseline geometry
- this repeats the earlier lesson from the hard `rel_z` threshold attempt: accepted `rel_z` is informative, but pushing it too high causes `terminal_smoke` regressions

## Conclusion
- Keep the new `rel_z` soft-penalty **infrastructure**.
- Do **not** enable it by default in `terminal_smoke` yet.
- The initial `3.0 / 1.0` tuning is rejected.
- Profile defaults were reverted to:
  - `rear_entry_prediction_score_min_rel_z_target_m=0.0`
  - `rear_entry_prediction_score_min_rel_z_weight=0.0`

## Practical takeaway
- `accepted_window_rel_z` is still the strongest candidate discriminator found so far.
- But on the correct carrier-prehold baseline, its practical operating band is lower and more variable than the first probe assumed.
- The next discriminator pass should therefore be:
  1. gather more **correct-baseline** accepted-window samples
  2. compare `rel_z` jointly with:
     - `phase_err_deg`
     - `ahead`
     - `pred_score`
  3. only then try a milder soft band, not another near-hard veto

## Next recommended step
- Leave the new hook disabled by default.
- Continue the short-vs-long-family work by adding one more **non-brittle** discriminator, likely based on:
  - a milder `rel_z` preference after more data, or
  - explicit global-candidate availability / route-feasibility information, since current accepted smoke windows are still pure local-gate accepts (`int_* = nan`).
