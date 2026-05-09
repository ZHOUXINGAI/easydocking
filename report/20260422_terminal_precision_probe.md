# 2026-04-22 terminal precision probe

## Goal
- keep the new short `M1` release family
- reduce `DOCKING` endgame drift where the carrier stays too far ahead
- collapse terminal `z` sooner instead of hovering high inside the loose band

## Code changes tested
- `src/easydocking_control/src/docking_controller.cpp`
  - in non-corridor `DOCKING`, added a near-field **ahead-bleed** behavior:
    - when the carrier is already ahead and terminal geometry is reasonably aligned,
      allow the along-track command floor to drop below the mini along speed
    - shrink the near-terminal ahead margin instead of keeping the carrier artificially far ahead
  - added a near-field **staged target collapse**:
    - when distance/lateral/band conditions are good enough, reduce the staged `z` and along offsets faster
    - this moves the controller earlier toward the real terminal target instead of lingering on the higher staged target

## Focused runs

### `20260422_012536_px4_sih`
- `classification=geometry-fail`
- short release family preserved:
  - `time_to_first_tracking_sec=5.460`
  - `time_to_first_docking_sec=9.700`
  - `post_start_path_length_m=103.618`
- terminal behavior:
  - `min_distance_m=1.268`
  - `min_abs_along_in_docking ≈ 0.729`
  - `min_abs_lat_in_docking ≈ 0.174`
  - `min_abs_zerr_in_docking ≈ 0.344`
- interpretation:
  - this is still not enough for capture, but compared with the older bad family it does show the along gap being reduced into the `~0.7–1.0m` range instead of staying much larger

### `20260422_012944_px4_sih`
- `classification=geometry-fail`
- this run regressed to the old long release family:
  - `time_to_first_tracking_sec=15.160`
  - `post_start_path_length_m=294.100`
- terminal-side observation is still useful:
  - `min_abs_along_in_docking ≈ 0.647`
  - `min_abs_zerr_in_docking ≈ 0.062`
- interpretation:
  - the staged-target collapse does help `z` come down much closer to the real terminal target
  - but along closure is still the dominant miss when the geometry is otherwise close

### `20260422_014021_px4_sih`
- short release family returned:
  - `time_to_first_tracking_sec=5.100`
  - `post_start_path_length_m=123.531`
- but it never entered `DOCKING`
- closest `TRACKING` geometry was still dominated by large lateral error (`|lat| ≈ 2.1m`)
- interpretation:
  - current bottleneck is not only inside `DOCKING`
  - the `TRACKING -> DOCKING` handoff is still inconsistent when lateral convergence stalls near the corridor

### `20260422_013348_px4_sih`
- `classification=start-window-fail`
- interpretation:
  - auto-start variability is now contaminating terminal validation
  - this needs to be treated as a separate regression source, not mixed into controller conclusions

## Current conclusion
- keep the new terminal changes for now:
  - they are directionally useful
  - especially for reducing excessive terminal `z`
- the main remaining precision bottleneck is still **along closure**
- there is now a second validation blocker:
  - **auto-start noise** is making terminal probes non-deterministic

## Recommended next step
1. keep the current release-family fix in `scripts/wait_for_docking_window.py`
2. continue terminal work with two narrow targets:
   - stronger near-field along-gap bleed-off once `lat/z` are acceptable
   - cleaner `TRACKING -> DOCKING` handoff so `DOCKING` does not start with large residual lateral error
3. add a more deterministic terminal smoke path so controller changes can be evaluated without auto-start randomness dominating the result

## Update: close-range TRACKING / DOCKING probe

### Additional code changes
- `src/easydocking_control/src/docking_controller.cpp`
  - added close-range `TRACKING` **terminal lateral recover**:
    - only active when `z` is already in band, distance is small, and lateral error is still large
    - gives lateral correction more velocity budget and less smoothing so `TRACKING` can actually hand off into `DOCKING`
  - strengthened non-corridor `DOCKING` **ahead-gap bleed** further:
    - activates earlier
    - allows larger temporary underspeed versus the mini to remove excessive ahead-gap near terminal capture

### `20260422_103540_px4_sih`
- short release family preserved:
  - `time_to_first_tracking_sec=5.240`
  - `post_start_path_length_m=106.795`
- `TRACKING -> DOCKING` handoff improved:
  - entered `DOCKING` at about `distance=1.83m`
  - closest `TRACKING` samples reached `|lat| ≈ 1.64m` with stronger lateral closing than the earlier stuck case
- still failed terminal completion:
  - `min_distance_m=1.640`
  - `min_abs_along_in_docking ≈ 0.641`
  - later in `DOCKING`, along drifted back to about `-2.8m`
- interpretation:
  - the new `TRACKING` lateral-recover logic is useful
  - the remaining miss is now even more clearly a near-terminal along-gap problem

### `20260422_103942_px4_sih`
- this run fell into the old long release family:
  - `time_to_first_tracking_sec=15.700`
  - `post_start_path_length_m=315.710`
- but terminal controller did complete:
  - `classification=final-pass`
  - `final_distance_m=0.200`
  - `final_rel_z_m=0.200`
  - `final_pass_v1_hold_sec=0.840`
- interpretation:
  - terminal-side changes are now strong enough to achieve real capture in at least one run
  - however, start / release-family variability is still large enough that terminal validation can easily be dominated by the wrong intercept family

### `20260422_103227_px4_sih`
- fixed `START_DELAY=78` was tested as a supposed deterministic smoke path
- result was not reliable enough:
  - `classification=geometry-fail`
  - `post_start_path_length_m=78.992`
  - closest `TRACKING` geometry still had `|lat| ≈ 3.6–3.9m`
- conclusion:
  - a fixed wall-clock start is **not** a trustworthy deterministic terminal smoke path because startup timing jitter changes the orbit phase too much

## Updated conclusion
- the controller bottleneck has narrowed:
  - close-range `TRACKING` handoff is better than before
  - terminal `DOCKING` can now complete in at least one observed run
- the remaining engineering problem is now split cleanly into two tracks:
  1. preserve the short `M1` release family consistently
  2. keep improving near-terminal along-gap removal on that short family
- validation also needs a **gate-based** deterministic terminal smoke path, not a fixed-time start delay

## Update: 2026-04-23 terminal smoke path

### Gate-path fixes
- `scripts/wait_for_docking_window.py`
  - fixed `rear_entry_elapsed_after_orbit_sec` so it is computed whenever orbit completion is known, not only when the energy timing gate is enabled
  - made orbit-progress release use `abs(accumulated_angle)` so the gate does not depend on orbit sign
- `scripts/run_px4_sih_docking_experiment.sh`
  - fixed `terminal_smoke` so the profile can really override `rear_entry_min_orbit_progress_ratio`
  - first tried a strict **global-primary-only** smoke path:
    - `require_global_primary_gate=true`
    - `secondary_gate=false`
    - `energy_timing=false`
    - `min_orbit_progress_ratio=0.25`

### Strict global-primary result

#### `20260423_174525_px4_sih`
- `classification=start-window-fail`
- evidence:
  - profile override was active (`experiment_duration_sec=180`, `min_orbit_progress_ratio=0.25`)
  - after orbit progress / completion, no acceptable global-primary gate reappeared
- interpretation:
  - global-primary-only is still too strict for a practical terminal smoke path
  - it is useful as a probe, but not as the default validation helper

### Tight local-primary-after-progress result

The next A/B used:
- `require_global_primary_gate=false`
- keep `secondary_gate=false`
- keep `energy_timing=false`
- `min_orbit_progress_ratio=0.25`
- tighten local primary gate to:
  - `prediction_score_threshold=0.9`
  - `prediction_lateral_max_m=8.0`

#### `20260423_175711_px4_sih`
- `classification=final-pass`
- accepted at:
  - `start_t_sec=26.480`
  - `start_intercept_pred_score=0.746`
- completion:
  - `first_completed_t_sec=49.180`
  - `final_pass_v1_hold_sec=1.200`
- note:
  - this validates that the gate-based smoke path can now leave `IDLE` deterministically and reach real capture

#### `20260423_180005_px4_sih`
- `classification=final-pass`
- accepted at:
  - `start_t_sec=36.340`
  - `start_intercept_pred_score=0.632`
- completion:
  - `first_completed_t_sec=78.440`
  - `final_pass_v1_hold_sec=0.840`
- note:
  - the smoke path is now reproducible enough for controller probing, but the release family is still not consistently short

### Current conclusion
- we now have a **working gate-based deterministic terminal smoke path**
- the practical version is:
  - orbit-progress release at `0.25` lap
  - no secondary gate
  - no energy timing gate
  - tight local primary gate (`score<=0.9`, `lateral<=8m`)
- this solves the old “fixed `START_DELAY` / random window” validation blocker
- the next problem is narrower:
  - keep this smoke path available for terminal-controller iteration
  - then reduce its long-tail path variability so smoke validation is not only deterministic, but also closer to the short intercept family
