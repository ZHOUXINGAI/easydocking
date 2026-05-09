# 2026-04-24 Terminal Smoke Baseline Batch (5 runs)

## Goal
- Re-check the **correct carrier-prehold baseline** after the rejected `rel_z` soft-tuning attempt.
- Confirm whether `terminal_smoke` is back in a healthy state with the new hook disabled by default.
- Collect more accepted-window samples before trying another discriminator tweak.

## Config
- `START_RVIZ=false`
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `rear_entry_prediction_score_min_rel_z_target_m=0.0`
- `rear_entry_prediction_score_min_rel_z_weight=0.0`

## Runs

| run | classification | post_start_path_length_m | docking_path_length_m | accepted_rel_z_m | accepted_phase_err_deg | accepted_ahead_m | accepted_pred_lat_m | accepted_pred_score |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `20260424_181703_px4_sih` | `final-pass` | `200.506` | `138.368` | `3.260` | `89.5` | `64.840` | `-2.650` | `0.730` |
| `20260424_181928_px4_sih` | `geometry-fail` | `390.841` | `330.475` | `3.260` | `90.8` | `66.300` | `-2.240` | `0.710` |
| `20260424_182316_px4_sih` | `final-pass` | `175.069` | `92.261` | `2.970` | `92.5` | `68.820` | `-1.730` | `0.670` |
| `20260424_182519_px4_sih` | `geometry-fail` | `1405.390` | `1386.958` | `2.790` | `92.7` | `68.850` | `-1.530` | `0.670` |
| `20260424_182910_px4_sih` | `geometry-fail` | `1675.757` | `` | `3.560` | `90.9` | `67.870` | `-3.670` | `0.800` |

## Aggregate
- `final-pass = 2/5`
- `geometry-fail = 3/5`
- short family:
  - `post_start_path_length_m ≈ 175–201`
- bad family:
  - `post_start_path_length_m ≈ 391–1676`

### Accepted-window means by class

**final-pass**
- `accepted_rel_z_m`: mean `3.115`
- `accepted_phase_err_deg`: mean `91.0`
- `accepted_ahead_m`: mean `66.83`
- `accepted_pred_lat_m`: mean `-2.19`
- `accepted_pred_score`: mean `0.70`

**geometry-fail**
- `accepted_rel_z_m`: mean `3.203`
- `accepted_phase_err_deg`: mean `91.47`
- `accepted_ahead_m`: mean `67.67`
- `accepted_pred_lat_m`: mean `-2.48`
- `accepted_pred_score`: mean `0.727`

## Main conclusion
- The repo is back in a **neutral / analyzable** state after reverting the aggressive `rel_z` tuning.
- But the correct-baseline `terminal_smoke` batch is still **not stable enough**:
  - only `2/5 final-pass`
  - long-tail failures are still very large

## Important observation
- The accepted-window fields still do **not** provide a clean scalar separator:
  - `accepted_rel_z_m` overlaps between good and bad runs
  - `accepted_phase_err_deg` overlaps
  - `accepted_ahead_m` overlaps
  - `accepted_pred_score` overlaps
- One fail (`20260424_182910_px4_sih`) is noticeably worse in:
  - `accepted_pred_lat_m=-3.670`
  - `accepted_pred_score=0.800`
  but the other two fails are close to the good runs, especially:
  - `20260424_182519_px4_sih`

## New M0 instrumentation check
- Added early post-start path-efficiency metrics:
  - `post_start_10s_path_length_m`
  - `post_start_10s_net_displacement_m`
  - `post_start_10s_path_efficiency_ratio`
  - `post_start_20s_*`
- Latest batch values:
  - `20260424_181703_px4_sih`: `10s=1.062`, `20s=1.041`
  - `20260424_181928_px4_sih`: `10s=1.133`, `20s=1.056`
  - `20260424_182316_px4_sih`: `10s=1.110`, `20s=1.068`
  - `20260424_182519_px4_sih`: `10s=1.165`, `20s=1.113`
  - `20260424_182910_px4_sih`: `10s=1.082`, `20s=1.147`

Interpretation:
- early path efficiency is directionally useful for the very bad family,
  but still not a clean one-shot separator:
  - `20260424_182519_px4_sih` and `20260424_182910_px4_sih` are clearly less efficient early
  - `20260424_181928_px4_sih` still overlaps with the good family
- so even early post-start shape alone is not yet enough to gate the family cleanly

## Engineering interpretation
- The current smoke release is still mostly selecting **similar-looking local windows** that later diverge into very different path families.
- That matches the existing instrumentation:
  - accepted smoke windows are still pure local-gate accepts
  - `accepted_window_int_* = nan`
- So the dominant missing discriminator is likely **not another tiny local threshold**, but better selection of the actual release geometry family.

## Recommended next step
- Do **not** blindly tighten `rel_z`, `phase_err`, or `pred_score`.
- Next probe should focus on one of:
  1. exposing more **global-intercept candidate diagnostics** at accepted time
  2. explicitly distinguishing whether the accepted local window corresponds to a short-family vs long-family future route
  3. combining the new early path-efficiency metrics with accepted-time geometry, instead of tightening any single local threshold
