# 2026-04-25 terminal_smoke first-timeout second-entry patch batch-5

## Goal

Validate whether the new first-timeout retry-state patch is:

- actually exercised online,
- able to recover first passive `DOCKING` timeout cases quickly,
- and safe enough for mainline use without reintroducing the bad repeated-loop family that killed the earlier `soft_vertical_stall` probe.

## Baseline

Common config:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.79`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.95`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `START_RVIZ=false`

Patch under test:

- `src/easydocking_control/src/docking_controller.cpp`
- on the first passive `DOCKING` timeout retrack, arm:
  - `passive_retry_used_ = true`
  - `passive_retry_pending_second_entry_ = true`

## Batch

Runs:

- `20260425_180156_px4_sih`
- `20260425_180553_px4_sih`
- `20260425_180755_px4_sih`
- `20260425_181018_px4_sih`
- `20260425_181220_px4_sih`

## Result

- `final-pass = 4/5`
- `start-window-fail = 1/5`
- no geometry-fail runs in this batch

Per-run:

| run | classification | post_start_path_length_m | docking_path_length_m | docking_entry_count | note |
|---|---|---:|---:|---:|---|
| `20260425_180156_px4_sih` | `start-window-fail` | — | — | `0` | never left `IDLE`; likely same silent starter anomaly family |
| `20260425_180553_px4_sih` | `final-pass` | `272.635` | `197.364` | `3` | first timeout redock in `0.44s` |
| `20260425_180755_px4_sih` | `final-pass` | `373.597` | `294.903` | `5` | still a long late-loop pass family |
| `20260425_181018_px4_sih` | `final-pass` | `248.883` | `166.472` | `3` | first timeout redock in `0.46s` |
| `20260425_181220_px4_sih` | `final-pass` | `192.642` | `128.841` | `2` | cleanest run in batch; first timeout redock in `0.50s` |

Pass-only averages:

- `post_start_path_length_m ≈ 271.9`
- `docking_path_length_m ≈ 196.9`
- `docking_entry_count ≈ 3.25`

## Timeout-redock behavior

This was the main thing under test.

Observed on all `4/4` passes:

- `20260425_180553`: first `DOCKING -> TRACKING` at `53.94s`, redock after `0.44s`
- `20260425_180755`: first `DOCKING -> TRACKING` at `47.70s`, redock after `0.50s`
- `20260425_181018`: first `DOCKING -> TRACKING` at `48.44s`, redock after `0.46s`
- `20260425_181220`: first `DOCKING -> TRACKING` at `55.80s`, redock after `0.50s`

This is the strongest evidence that the patch is doing the intended job:

- first passive timeout no longer falls back into a dead ordinary `TRACKING` path,
- it now consistently re-enters `DOCKING` quickly through the existing retry machinery.

## What is improved

- the original state-machine gap seen in `20260425_141103_px4_sih` is now addressed online
- first-timeout recovery is no longer hypothetical; it is visible in repeated fresh runs
- this looks cleaner than the reverted `soft_vertical_stall` trigger because it reuses existing retry state instead of widening the retry geometry surface

## What is still not solved

- headline batch rate did **not** improve beyond the current `4/5` class, because one run still fell into `start-window-fail`
- the patch does **not** remove the later long-tail family by itself:
  - `20260425_180755_px4_sih` still needed `5` docking entries
  - path length remained high (`post_start_path_length_m = 373.597`)

So this patch fixes the **first-timeout recovery gap**, but it does not yet solve:

- the occasional starter anomaly
- repeated later `DOCKING -> TRACKING` loops once the first recovery has already happened

## Mainline judgment

- keep this first-timeout second-entry patch
- treat it as a valid controller-side improvement
- do **not** claim M3 closure from it

## Next step

Focus should now split cleanly:

1. starter anomaly:
   - investigate the `never_left_idle` / silent `start_command.log` family separately
2. terminal long-tail:
   - reduce the later repeated `DOCKING -> TRACKING` loops without broadening retry triggers again
   - especially compare the long pass `20260425_180755_px4_sih` against the cleaner `20260425_181220_px4_sih`
