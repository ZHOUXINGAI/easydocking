# 2026-04-23 Terminal Smoke Batch (5 runs)

## Goal
- Verify that the new `terminal_smoke` profile is now a usable deterministic terminal-validation path.
- Measure whether it is only stable in “leave `IDLE` + complete capture”, or also stable in path length / release family.

## Config
- `CARRIER_ACTIVATE_ON_LAUNCH=true`
- `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- `AUTO_START_WINDOW_PROFILE=terminal_smoke`
- `START_RVIZ=false`

Profile defaults in this batch:
- `rear_entry_min_orbit_progress_ratio=0.25`
- `rear_entry_require_global_primary_gate=false`
- `rear_entry_prediction_secondary_gate_enabled=false`
- `rear_entry_enable_energy_timing_gate=false`
- `rear_entry_prediction_score_threshold=0.9`
- `rear_entry_prediction_lateral_max_m=8.0`

## Runs

| run | classification | start_t_sec | first_completed_t_sec | start_to_completed_sec | post_start_path_length_m | docking_path_length_m | start_intercept_pred_score | final_pass_v1_hold_sec |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `20260423_192026_px4_sih` | `final-pass` | 33.180 | 85.620 | 52.440 | 396.916 | 312.565 | 0.616 | 1.200 |
| `20260423_192252_px4_sih` | `final-pass` | 33.240 | 51.100 | 17.860 | 117.549 | 45.310 | 0.636 | 0.600 |
| `20260423_192443_px4_sih` | `final-pass` | 33.380 | 82.980 | 49.600 | 375.358 | 290.550 | 0.647 | 1.060 |
| `20260423_192705_px4_sih` | `final-pass` | 33.300 | 76.040 | 42.740 | 318.446 | 243.376 | 0.691 | 0.860 |
| `20260423_192918_px4_sih` | `final-pass` | 33.080 | 50.540 | 17.460 | 113.793 | 46.637 | 0.643 | 1.240 |

## Aggregate
- `final-pass`: `5/5`
- `start_t_sec`: mean `33.236`, min `33.080`, max `33.380`
- `time_to_first_tracking_sec`: mean `5.832`, min `5.700`, max `6.000`
- `time_to_first_docking_sec`: mean `12.992`, min `12.160`, max `13.700`
- `first_completed_t_sec`: mean `69.256`, min `50.540`, max `85.620`
- `start_to_completed_sec`: mean `36.020`, min `17.460`, max `52.440`
- `post_start_path_length_m`: mean `264.412`, min `113.793`, max `396.916`
- `docking_path_length_m`: mean `187.688`, min `45.310`, max `312.565`
- terminal quality stayed tight:
  - `final_abs_xy_max_m`: mean `0.013`, max `0.034`
  - `final_rel_z_m`: mean `0.200`
  - `final_pass_v1_hold_sec`: mean `0.992`, min `0.600`

## Main conclusion
- The deterministic smoke-path objective is achieved:
  - release timing is highly repeatable (`~33.2s`)
  - completion is repeatable (`5/5 final-pass`)
- The remaining issue is no longer “can it start / can it finish”.
- The remaining issue is **release-family bifurcation after the same smoke gate**:
  - short family: `post_start_path_length_m ≈ 114–118m`
  - long family: `post_start_path_length_m ≈ 318–397m`

## Interpretation
- `terminal_smoke` is now good enough to use as a controller-validation entry path.
- It is not yet good enough to be the final “clean short-family probe”.
- The next work item should be:
  - keep the current smoke gate stable,
  - then add one more geometry discriminator so accepted runs prefer the short family instead of merely any eventual completion family.

## Suggested next step
- Compare the accepted intercept geometry between:
  - short-family runs: `20260423_192252_px4_sih`, `20260423_192918_px4_sih`
  - long-family runs: `20260423_192026_px4_sih`, `20260423_192443_px4_sih`, `20260423_192705_px4_sih`
- Candidate next discriminator to test:
  - tighter accepted `prediction_along_m` band
  - tighter accepted global route distance / forward-cos
  - an explicit path-length surrogate in the release gate, not only prediction score
