# 2026-04-25 terminal_smoke narrow pred-margin probe

## Goal

After reverting the speculative controller-side relapse branch, the main blocker moved back to the release side:

- residual bad runs were again dominated by the early `start_t ≈ 21–27s` family
- those runs never even reached the controller family we were trying to refine

The repo already had a smoke-only reject hook:

- `rear_entry_smoke_reject_prediction_score_min`
- `rear_entry_smoke_reject_diag_margin_max_mps`

So the next step was to test a **narrower env-only discriminator** instead of writing more controller logic.

## Hypothesis

Recent early-start fails clustered around:

- `phase_err_deg ≈ 84.7–86.8`
- `pred_lat ≈ -4.6 to -5.0`
- `pred_score ≈ 0.80–0.82`
- `diag_margin ≈ 1.58–1.63`

while nearby good runs were more like:

- `phase_err_deg ≈ 90.6–93.1`
- `pred_lat ≈ -1.3 to -2.5`
- `pred_score ≈ 0.62–0.74`
- `diag_margin ≈ 2.86–3.45`

So the probe used:

- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.79`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.95`

## Validation batch (`5-run`)

Common config:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `START_RVIZ=false`

Runs:

| run | classification | start_t_sec | phase_err_deg | pred_score | diag_margin | post_start_path_length_m | note |
|---|---|---:|---:|---:|---:|---:|---|
| `20260425_140622_px4_sih` | `final-pass` | `63.44` | `88.9` | `0.74` | `3.14` | `229.983` | late, but clean |
| `20260425_140902_px4_sih` | `final-pass` | `31.56` | `92.0` | `0.62` | `2.86` | `128.560` | short family |
| `20260425_141103_px4_sih` | `geometry-fail` | `36.88` | `92.9` | `0.70` | `3.38` | `1150.896` | residual tracking family |
| `20260425_141457_px4_sih` | `final-pass` | `37.14` | `93.1` | `0.65` | `3.45` | `144.404` | short family |
| `20260425_141710_px4_sih` | `final-pass` | `34.66` | `92.9` | `0.62` | `3.30` | `282.906` | long pass family |

## Result

- `final-pass = 4/5`
- no runs fell into the old `start_t ≈ 21–27s` early-start family
- accepted windows shifted back toward:
  - `start_t ≈ 31–37s` for the normal family
  - or a later clean family around `63–65s`

## Main interpretation

This probe did what it was supposed to do:

- it suppressed the targeted early release family
- it did **not** reintroduce the very early `84–87° / pred_score≈0.8 / diag_margin≈1.6` accepts

The remaining fail (`20260425_141103`) is important because it changes the bottleneck again:

- accepted window looks healthy
- `start_t=36.88`
- `phase_err_deg=92.9`
- `pred_score=0.70`
- `diag_margin=3.38`
- but the run still falls into a `TRACKING -> DOCKING -> TRACKING` long-tail family

So after this release-side improvement, the dominant blocker shifts back to **controller-side terminal / tracking quality**, not early start-window selection.

## Promotion

Because the discriminator is already implemented in repo and the batch result is healthy enough, these values were promoted into the `terminal_smoke` default profile in:

- `scripts/run_px4_sih_docking_experiment.sh`

New defaults:

- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.79`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.95`

## Post-promotion sanity

Two direct smoke runs were used to sanity-check the defaulted profile:

- `20260425_142230_px4_sih`
  - `start-window-fail`
  - notable anomaly:
    - `start_command.log` stops after orbit completion, with no later `window_check` lines
  - this does **not** look like a normal reject-by-threshold failure
- `20260425_143114_px4_sih`
  - `final-pass`
  - `start_t_sec=64.74`
  - `accepted_phase_err_deg=89.7`
  - `accepted_pred_score=0.68`
  - `accepted_diag_margin=2.79`

## Practical conclusion

- keep the new narrow pred-margin defaults in `terminal_smoke`
- monitor the occasional silent starter timeout separately
- with the early-start family suppressed again, the next mainline step can return to:
  - the residual `TRACKING / DOCKING` long-tail family
  - not more release-side threshold churn first
