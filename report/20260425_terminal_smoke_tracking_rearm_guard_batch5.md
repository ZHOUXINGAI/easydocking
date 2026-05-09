# 2026-04-25 `terminal_smoke` early-`TRACKING` re-arm guard probe (5 runs)

## Goal

- keep the proven release baseline unchanged:
  - `CARRIER_ACTIVATE_ON_LAUNCH=true`
  - `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
  - `AUTO_START_WINDOW_PROFILE=terminal_smoke`
  - `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- add one **controller-side**, **default-off** probe only:
  - `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- test whether the new guard improves the remaining long-tail `TRACKING` family without retuning terminal control

## Guard implemented

The new guard is intentionally narrow:

- only active in passive mode
- only active before the first `DOCKING` entry
- at most one re-arm per run
- action: early `TRACKING -> APPROACH` re-arm, then let the normal intercept path rebuild

Current probe thresholds:

- Branch A (`front-cross wide`, first `5s`)
  - `min_terminal_distance < 3.0`
  - `min_lateral_abs > 1.6`
  - `max_along_error > 1.0`
- Branch B (`tracking stall`, first `8s`)
  - `min_terminal_distance > 2.5`
  - `min_lateral_abs > 2.2`
  - `max_along_error < 0.2`

## Config

- `START_RVIZ=false`
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`

## Fresh batch

| run | classification | start_t_sec | post_start_path_length_m | docking_path_length_m | phase sequence |
|---|---:|---:|---:|---:|---|
| `20260425_000334_px4_sih` | `final-pass` | `33.40` | `102.015` | `82.601` | `IDLE → APPROACH → TRACKING → DOCKING → TRACKING → DOCKING → COMPLETED` |
| `20260425_000529_px4_sih` | `final-pass` | `34.34` | `220.728` | `201.402` | `IDLE → APPROACH → TRACKING → DOCKING → TRACKING → DOCKING → TRACKING → DOCKING → COMPLETED` |
| `20260425_000748_px4_sih` | `final-pass` | `33.54` | `139.973` | `72.092` | `IDLE → APPROACH → TRACKING → DOCKING → COMPLETED` |
| `20260425_000951_px4_sih` | `final-pass` | `24.94` | `86.670` | `33.256` | `IDLE → APPROACH → TRACKING → DOCKING → COMPLETED` |
| `20260425_001157_px4_sih` | `final-pass` | `33.36` | `156.802` | `73.375` | `IDLE → APPROACH → TRACKING → DOCKING → COMPLETED` |

## Result

- `final-pass = 5/5`
- `start_prehold_ready = 5/5`
- no `start-window-fail`
- no observed early `TRACKING -> APPROACH` re-arm in this batch

## Important interpretation

This batch is **good news**, but it is **not yet proof that the new guard solved the residual family**.

Reason:

- the 5 fresh runs all passed
- but none of them actually exercised the new re-arm path
- so the observed `5/5` is a successful non-regression batch, not yet a causal demonstration of the guard

## Offline rule exercise on known historical slice

Using the same thresholds on the known `20260424` slice:

### Intended fail-family hits

- `20260424_220319_px4_sih`
  - Branch A hit
- `20260424_221224_px4_sih`
  - Branch A hit
- `20260424_231539_px4_sih`
  - Branch B hit

### No false positives on representative passes

- `20260424_220951_px4_sih`
  - no hit
- `20260424_220717_px4_sih`
  - no hit
- `20260424_221627_px4_sih`
  - no hit

### No false positives on this fresh `5/5` batch

- `20260425_000334_px4_sih`
  - no hit
- `20260425_000529_px4_sih`
  - no hit
- `20260425_000748_px4_sih`
  - no hit
- `20260425_000951_px4_sih`
  - no hit
- `20260425_001157_px4_sih`
  - no hit

## Practical conclusion

- keep the guard in repo as **default-off infrastructure**
- current probe passes the first bar:
  - build is clean
  - no fresh false-positive regressions
  - offline rule still matches the intended historical fail family
- do **not** promote it to default yet
- next useful validation is:
  - either a larger fresh batch until the bad family reappears,
  - or a targeted probe that increases the chance of hitting the old `TRACKING` fail family
