# 2026-04-21 M1 Release Gate Batch (5 runs)

## Scope
- config: `CARRIER_ACTIVATE_ON_LAUNCH=true`
- config: `CARRIER_IDLE_HOVER_ALTITUDE=29.4`
- config: `AUTO_START_WINDOW_PROFILE=prehold`
- code state: global-intercept-first release gate with route-to-intercept primary acceptance

## Batch result

| run | classification | start_t_s | post_start_path_m | docking_path_m |
| --- | --- | ---: | ---: | ---: |
| `20260421_144738_px4_sih` | `geometry-fail` | 54.780 | 275.054 | 140.925 |
| `20260421_145006_px4_sih` | `final-pass` | 55.640 | 196.656 | 67.356 |
| `20260421_145235_px4_sih` | `final-pass` | 64.640 | 179.092 | 118.416 |
| `20260421_145502_px4_sih` | `geometry-fail` | 66.240 | 181.916 | 64.388 |
| `20260421_145734_px4_sih` | `geometry-fail` | 69.640 | 143.904 | 30.219 |

## Summary
- release gate starts `5/5`; no run stayed in `IDLE`
- `FINAL_PASS`: `2/5`
- best completion run: `20260421_145006_px4_sih`
- best short docking leg: `20260421_145734_px4_sih` with `docking_path_length_m=30.219`, but terminal criteria still failed
- main remaining problem is no longer release-gate deadlock; it is path efficiency / terminal quality after `START`

## Takeaway
- the M1 release-gate integration is now functionally working
- next tuning should focus on reducing the very long `post_start_path_length_m` outliers and lifting `geometry-fail` runs into stable `FINAL_PASS`
