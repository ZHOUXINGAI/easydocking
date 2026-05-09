# Energy Constraint + 5-run Validation

## Reference from this week (short chase)
Selected references: shortest carrier chase-path among current `prediction_primary + ahead=front` runs.

| Run | first_non_idle_t (s) | carrier_path_to_completed (m) | approach_duration (s) | accepted dist_xy (m) | pre-accept ahead (m) |
|---|---:|---:|---:|---:|---:|
| `20260411_171246_px4_sih` | 79.100 | 167.33 | 23.90 | 93.99 | 81.97 |
| `20260411_170150_px4_sih` | 78.300 | 170.36 | 24.54 | 97.55 | 86.28 |

Derived timing target: release around `orbit_done + 26s` (reference window ~26.0-26.8s).

## Added constraint
- `rear_entry_enable_energy_timing_gate=true`
- `rear_entry_energy_min_after_orbit_sec=26.0`
- `rear_entry_energy_max_after_orbit_sec=40.0`

This delays START until the energy window is reached, to avoid long chase legs.

## New 5-run results
| Run | Classification | first_non_idle_t (s) | START reason | accepted dist_xy (m) | accepted pred_along | accepted pred_lat | pre-accept ahead | pre-accept ahead_ok | pre-accept energy_after_orbit (s) | path_to_completed (m) | approach_duration (s) |
|---|---|---:|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `20260411_173229_px4_sih` | final-pass | 78.680 | window | 95.30 | -57.78 | -17.23 | 86.06 | 1 | 25.55 | 183.97 | 26.06 |
| `20260411_173517_px4_sih` | final-pass | 77.541 | window | 95.94 | -59.72 | -18.26 | 87.05 | 1 | 25.60 | 195.74 | 27.50 |
| `20260411_173806_px4_sih` | final-pass | 77.841 | window | 73.86 | -49.92 | 2.04 | 75.95 | 1 | 25.40 | 192.75 | 26.90 |
| `20260411_174053_px4_sih` | final-pass | 77.800 | window | 74.78 | -51.08 | 5.33 | 76.93 | 1 | 25.45 | 145.92 | 20.90 |
| `20260411_174332_px4_sih` | final-pass | 78.000 | window | 95.38 | -59.09 | -18.73 | 86.86 | 1 | 25.60 | 159.68 | 23.10 |

## Before vs After (5-run mean)
| Metric | Before (no energy timing gate) | After (energy timing gate) | Delta |
|---|---:|---:|---:|
| first_non_idle_t (s) | 76.936 | 77.972 | +1.036 |
| approach_duration (s) | 25.18 | 24.89 | -0.28 |
| carrier_path_to_completed (m) | 178.75 | 175.61 | -3.13 |
| carrier_path_min/max (m) | 167.33 / 194.80 | 145.92 / 195.74 | - |
