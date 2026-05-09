# 2026-04-25 early-`TRACKING` re-arm effectiveness batch (`5-run`)

## Goal

After making the guard-triggered re-arm visible with a short `APPROACH` hold, the next question was:

- does the guard now trigger in real runs,
- and when it triggers, does it actually help outcome / path length

This batch keeps the proven `terminal_smoke` baseline and measures **trigger usefulness**, not just observability.

## Config

- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `AUTO_START_REAR_ENTRY_MIN_ORBIT_PROGRESS_RATIO=0.40`
- `TRACKING_ENTRY_REARM_GUARD_ENABLED=true`
- `START_RVIZ=false`

## Runs

| run | classification | phase sequence | guard_triggered | start_t_sec | post_start_path_length_m | docking_path_length_m |
|---|---|---|---:|---:|---:|---:|
| `20260425_020031_px4_sih` | `final-pass` | `IDLE -> APPROACH -> TRACKING -> APPROACH -> TRACKING -> DOCKING -> COMPLETED` | `1` | `62.20` | `131.510` | `62.185` |
| `20260425_020254_px4_sih` | `final-pass` | `IDLE -> APPROACH -> TRACKING -> DOCKING -> TRACKING -> DOCKING -> TRACKING -> DOCKING -> COMPLETED` | `0` | `67.94` | `271.062` | `204.853` |
| `20260425_020535_px4_sih` | `geometry-fail` | `IDLE -> APPROACH -> TRACKING -> DOCKING -> TRACKING` | `0` | `36.94` | `1153.251` | `1090.455` |
| `20260425_020925_px4_sih` | `final-pass` | `IDLE -> APPROACH -> TRACKING -> DOCKING -> TRACKING -> DOCKING -> TRACKING -> DOCKING -> TRACKING -> DOCKING -> COMPLETED` | `0` | `33.60` | `296.435` | `216.654` |
| `20260425_021133_px4_sih` | `final-pass` | `IDLE -> APPROACH -> TRACKING -> APPROACH -> TRACKING -> DOCKING -> COMPLETED` | `1` | `36.60` | `127.603` | `68.655` |

## Batch summary

- `final-pass = 4/5`
- observed guard-trigger count = `2/5`
- triggered runs:
  - `final-pass = 2/2`
  - average `post_start_path_length_m = 129.556`
  - average `docking_path_length_m = 65.420`
- non-triggered runs:
  - `final-pass = 2/3`
  - average `post_start_path_length_m = 573.583`
  - average `docking_path_length_m = 503.987`

## What this batch says

There is now real online evidence that the guard can be both:

- **visible**
- **useful**

In this batch, both triggered runs converted to `final-pass`, and both stayed in the short-path family.

That is stronger than the earlier hold probe, where visibility existed but usefulness was still mixed.

## What this batch does **not** solve

The remaining bad run is important:

- `20260425_020535_px4_sih`
- phase sequence: `IDLE -> APPROACH -> TRACKING -> DOCKING -> TRACKING`
- no guard-triggered `TRACKING -> APPROACH`
- still collapsed into a very long path (`1153m`)

So the residual family is now split into at least two cases:

1. **late first-entry `TRACKING` long-tail**, where the new re-arm can help
2. **early `TRACKING -> DOCKING -> TRACKING` relapse**, which is outside the current “first pre-`DOCKING` `TRACKING` entry only” policy

This means the current guard is no longer just observability infrastructure. It has real value, but its **coverage is incomplete**.

## Practical conclusion

- keep the re-arm guard in repo
- keep it **default-off** for now
- treat the current result as:
  - **promising effectiveness**
  - but **not full closure**

## Recommended next step

Do **not** go back to accepted-window tuning first.

The next controller-side discriminator should focus on the remaining unhandled family:

- either tighten the `TRACKING -> DOCKING` admission so `20260425_020535`-style runs do not enter `DOCKING` prematurely
- or extend the re-arm policy to cover the first **post-`DOCKING` relapse into `TRACKING`**

That is now the highest-value path to remove the last long-tail family without disturbing the working short-family baseline.
