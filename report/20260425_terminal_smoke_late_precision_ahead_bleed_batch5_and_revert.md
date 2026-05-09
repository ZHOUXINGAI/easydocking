# 2026-04-25 terminal_smoke late-precision ahead-bleed batch-5 and revert

## Goal

Confirm whether the late-entry ahead-bleed probe from the earlier `3-run` smoke is strong enough to keep in mainline.

The probe had looked promising in:

- `20260425_183530`
- `20260425_183728`
- `20260425_183929`

But that was only smoke-scale.

## Confirmation batch

Common baseline:

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_PREDICTION_SCORE_MIN=0.79`
- `AUTO_START_REAR_ENTRY_SMOKE_REJECT_DIAG_MARGIN_MAX_MPS=1.95`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `START_RVIZ=false`

Runs:

- `20260425_184435_px4_sih`
- `20260425_184633_px4_sih`
- `20260425_184910_px4_sih`
- `20260425_185135_px4_sih`
- `20260425_185344_px4_sih`

## Result

- `final-pass = 4/5`
- `geometry-fail = 1/5`
- no `start-window-fail`

Per-run:

| run | classification | post_start_path_length_m | docking_path_length_m | docking_entry_count | note |
|---|---|---:|---:|---:|---|
| `20260425_184435_px4_sih` | `final-pass` | `139.590` | `72.463` | `1` | clean |
| `20260425_184633_px4_sih` | `final-pass` | `219.224` | `147.833` | `2` | acceptable |
| `20260425_184910_px4_sih` | `final-pass` | `442.490` | `367.237` | `6` | late-loop family still present |
| `20260425_185135_px4_sih` | `final-pass` | `202.492` | `136.460` | `2` | acceptable |
| `20260425_185344_px4_sih` | `geometry-fail` | `1264.858` | — | `0` | never reached `DOCKING`; stuck in `TRACKING` |

Pass-only averages:

- `post_start_path_length_m ≈ 250.9`
- `docking_path_length_m ≈ 181.0`
- `docking_entry_count ≈ 2.75`

## Why this was rejected

The `3-run` smoke was not robust enough.

The confirmation batch shows mixed behavior:

- two short clean passes exist
- but the old late-loop family still survives:
  - `20260425_184910`
  - `docking_entry_count = 6`
  - `docking_path_length_m = 367.237`
- and a new `TRACKING`-only geometry fail also appears:
  - `20260425_185344`
  - phase sequence stayed at `IDLE -> APPROACH -> TRACKING`

So this patch does **not** show the kind of clear online benefit needed for mainline retention.

Headline rate also did not improve beyond the current `4/5` class.

## Decision

- revert the late-entry ahead-bleed probe
- keep the earlier first-timeout second-entry patch
- do **not** stack another controller-side shaping branch into mainline without stronger evidence

## Practical next step

Split the remaining work more cleanly:

1. `TRACKING`-only fail family
   - investigate `20260425_185344`
   - why did it never collapse into `DOCKING`?
2. later long-tail pass family
   - compare `20260425_184910` against short passes like `20260425_184435`
   - identify whether the remaining blocker is:
     - `TRACKING -> DOCKING` admission quality, or
     - late `DOCKING` monotonicity
