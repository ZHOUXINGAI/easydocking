# `terminal_smoke` pred-score + diag-margin probe (`2026-04-24`)

## Goal

Test a new **accepted-window proxy reject** using only existing runtime signals:

- reject if:
  - `prediction_score >= 0.77`
  - `global_intercept_relaxed_speed_margin_mps <= 1.99`

This was chosen because it cleanly matched three known bad early families in the recent `2026-04-24` sample slice without hitting any known pass sample in that slice.

## Probe setup

- `START_RVIZ=false`
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- baseline kept:
  - `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
  - carrier prehold gate enabled
- probe-only reject:
  - `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.77`
  - `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.99`

## Runs

| run | classification | start_t | post_start_path_length_m | note |
| --- | --- | ---: | ---: | --- |
| `20260424_231342_px4_sih` | `final-pass` | `33.54` | `107.149` | very good short-family pass |
| `20260424_231539_px4_sih` | `geometry-fail` | `64.26` | `1221.686` | late release, still long-tail fail |
| `20260424_231940_px4_sih` | `final-pass` | `36.70` | `202.044` | acceptable pass |
| `20260424_232202_px4_sih` | `final-pass` | `67.24` | `137.349` | pass, but release clearly too late |
| `20260424_232443_px4_sih` | `final-pass` | `62.70` | `165.298` | pass, but release clearly too late |

## Result

- `final-pass = 4/5`
- `geometry-fail = 1/5`

Compared to the current promoted baseline:

- baseline `orbit_progress=0.40` batch: `4/5 final-pass`
- this probe: `4/5 final-pass`

## Interpretation

- The reject hook does suppress some early bad-window candidates (`smoke_pred_reject=1` is active repeatedly in `start_command.log`).
- But it does **not** improve the mainline success rate.
- More importantly, it often delays release too much:
  - three runs start around `62–67s`
  - this is much later than the current practical smoke baseline
- One run (`20260424_231539_px4_sih`) still becomes a long-tail `TRACKING` fail even after the later release.

## Decision

- Keep the new hook as **neutral infrastructure only**.
- Do **not** promote this reject gate into `terminal_smoke` defaults.

## Main takeaway

This reinforces the current diagnosis:

- accepted-window-only tuning can suppress part of the bad family
- but the remaining blocker is now more about **post-release TRACKING family behavior** than a single accepted-window threshold
- the next useful step should likely use an **early-TRACKING online discriminator / re-arm policy**, not another stricter release-only gate
