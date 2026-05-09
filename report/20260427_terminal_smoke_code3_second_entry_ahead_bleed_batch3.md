# 2026-04-27 terminal_smoke: second-entry code3 ahead-bleed probe (3-run)

## Baseline

All runs used:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.74`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=2.00`

## What changed

Two actions:

1. dropped the unvalidated narrow `code9`-only corridor release hook
2. added a narrow **second-entry ahead-bleed** inside non-corridor `DOCKING`
   - scope: passive mode, `passive_docking_entry_count >= 2`
   - geometry: already ahead, lateral/z reasonably aligned, corridor already released
   - effect: allow stronger along-speed bleed while still preserving positive carrier-ahead margin

Implementation is in:

- `src/easydocking_control/src/docking_controller.cpp`

## Previous 3-run reference (before this patch)

Runs:

- `20260427_015630_px4_sih`
- `20260427_015847_px4_sih`
- `20260427_020102_px4_sih`

Result:

- `final-pass = 2/3`
- one `geometry-fail`
- one pass started very late (`start_t_sec = 60.0`), so the set was not cleanly comparable

Representative issue:

- the remaining bad family was again `entry1 -> code4 -> entry2 -> code3 ...`
- on the long/fail family, about `4s` after second-entry corridor release the ahead gap was still large:
  - `20260427_015630_px4_sih`: `along ≈ -3.24`, `dist ≈ 3.38`
  - `20260427_015847_px4_sih`: `along ≈ -3.24`, `dist ≈ 3.63`

## New validation batch

Runs:

- `20260427_020835_px4_sih`
- `20260427_021044_px4_sih`
- `20260427_021248_px4_sih`

Headline:

- `final-pass = 3/3`
- all three starts are back in the normal `~33.1-33.3s` band

Per run:

- `20260427_020835_px4_sih`
  - sequence: `entry1 -> code4 -> entry2 -> code3 -> entry3 -> entry4 -> code2 -> completed`
  - `start_t_sec = 33.24`
  - `first_completed_t_sec = 77.44`
  - `post_start_path_length_m = 335.953`
  - `docking_path_length_m = 241.155`
- `20260427_021044_px4_sih`
  - sequence: `entry1 -> code4 -> entry2 -> code3 -> entry3 -> entry4 -> code1 -> completed`
  - `start_t_sec = 33.14`
  - `first_completed_t_sec = 73.64`
  - `post_start_path_length_m = 301.526`
  - `docking_path_length_m = 220.163`
- `20260427_021248_px4_sih`
  - sequence: `entry1 -> code4 -> entry2 -> code3 -> entry3 -> entry4 -> entry5 -> code1 -> completed`
  - `start_t_sec = 33.34`
  - `first_completed_t_sec = 84.44`
  - `post_start_path_length_m = 391.603`
  - `docking_path_length_m = 298.399`

## Main effect

The patch does **not** remove the second-entry `code3` family yet.

But it does improve its geometry enough that the batch recovers to `3/3 final-pass` under the normal start band.

Compared to the pre-patch bad family, about `4s` after second-entry corridor release:

- `20260427_020835_px4_sih`: `along ≈ -1.17`, `dist ≈ 1.53`
- `20260427_021044_px4_sih`: `along ≈ -1.77`, `dist ≈ 1.93`
- `20260427_021248_px4_sih`: `along ≈ -1.43`, `dist ≈ 1.62`

So the second-entry ahead gap shrinks much faster than before, even though the controller still later exits that entry with `code3` and needs one or more re-entries.

## Interpretation

This is a **useful mainline improvement**:

- it removes the immediate regression risk from the speculative `code9` hook
- it restores `3/3 final-pass` on the normal `terminal_smoke` baseline
- it narrows the second-entry `code3` geometry substantially

But it is **not closure** yet:

- `code3` still fires in all `3/3` runs
- path length is still too long (`docking_path_length_m ≈ 220-298m`)
- extra `entry3/entry4/entry5` loops remain the next bottleneck

## Next target

Stay on the second-entry `code3` family.

The next narrow controller step should target:

- reducing or eliminating the `entry2 -> code3` retrack itself, or
- making the first post-`code3` re-entry complete directly without additional `entry4/entry5` loops

This should be done without reopening the old early-start / release-threshold churn.
