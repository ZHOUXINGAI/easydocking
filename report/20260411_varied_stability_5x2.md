# Varied Trajectory Stability Report (5x2)

Date: 2026-04-11

## Scenarios

- **Scenario A**: `MINI_ORBIT_CENTER=(-30,25)`, `MINI_ORBIT_START_PHASE_DEG=30`, `CARRIER_OUTSIDE_ANGLE_DEG=45`
- **Scenario B**: `MINI_ORBIT_CENTER=(42,18)`, `MINI_ORBIT_START_PHASE_DEG=260`, `CARRIER_OUTSIDE_ANGLE_DEG=160`
- Common gates: `START_RVIZ=true`, `AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true`

## Aggregate result

- Scenario A: `5/5 final-pass`
- Scenario B: `5/5 final-pass`
- Total: `10/10 final-pass`
- All 10 runs used `START reason=window`.

## Direction metrics (startup, carrier_speed>=1 m/s)

- `opp_ratio_6s_speedg1 = 0.0` in all 10 runs.
- Scenario A `dt_to_carrier_speed>=1m/s`: mean `3.996s` (min `3.940`, max `4.040`)
- Scenario B `dt_to_carrier_speed>=1m/s`: mean `3.844s` (min `3.700`, max `4.000`)

## Signed front/back distance metrics

Definition: signed distance is projection of `(carrier-mini)` on mini velocity direction; positive means carrier ahead.

- **Scenario A**
  - `ahead_ratio_positive_pre_completed`: mean `0.251`
  - `ahead_min_m`: mean `-37.247 m` (worst `-39.577 m`)
  - `ahead_sign_flip_count`: mean `3.4`
- **Scenario B**
  - `ahead_ratio_positive_pre_completed`: mean `0.356`
  - `ahead_min_m`: mean `-10.373 m` (worst `-10.898 m`)
  - `ahead_sign_flip_count`: mean `2.0`

Interpretation: although final docking is stable (10/10), pre-completed phase still has frequent carrier-behind intervals (signed distance crosses negative).

## Energy gate behavior

- New auto-relax path is active (`rear_entry_energy_allow_max_relaxation=true`).
- In this 5x2 batch, `energy_relaxed=1` for all runs, i.e., START windows were accepted after relaxing energy max timing when prediction gate remained good.

## Artifacts

- Metrics CSV: `report/20260411_varied_stability_5x2_metrics.csv`
- Per-run GIF: `results/<RUN_ID>/trajectory_xy_full.gif`
- Runs:
  - Scenario A: `20260411_212917_px4_sih`, `20260411_213254_px4_sih`, `20260411_213625_px4_sih`, `20260411_213957_px4_sih`, `20260411_214329_px4_sih`
  - Scenario B: `20260411_220128_px4_sih`, `20260411_220456_px4_sih`, `20260411_220822_px4_sih`, `20260411_221148_px4_sih`, `20260411_221511_px4_sih`
