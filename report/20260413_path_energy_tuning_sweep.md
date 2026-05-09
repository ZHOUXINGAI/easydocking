# Path / Energy Tuning Sweep (2026-04-13)

Goal: reduce carrier path/time while keeping `final-pass 5/5` and not degrading ahead quality vs baseline.

| candidate | config | final_pass | docking_ahead_ratio_mean | docking_ahead_min_worst_m | path_mean_m | decision |
|---|---|---:|---:|---:|---:|---|
| baseline_r4e_tuned | 14.5 + ahead-release tuned (kept baseline) | 5/5 | 0.659 | -1.593 | 147.38 | reference |
| candA_no_early | 14.5 + early-release disabled | 2/2 | 0.473 | -1.945 | 132.43 | reject_ahead_drop |
| candB_early_plus2s | 14.5 + early-release min_after_orbit=14 | 2/2 | 0.501 | -1.465 | 124.94 | reject_ahead_drop |
| candE_speed14p2 | 14.2 speed limit | 1/2 | 0.769 | -1.485 | 448.25 | reject_stability |
| candD_speed13p8 | 13.8 speed limit | 5/5 | 0.555 | -1.785 | 134.08 | reject_ahead_drop |
| candCatchup_tuned | 14.5 + corridor_catchup 0.20->0.16 | 2/2 | 0.402 | -2.193 | 131.65 | reject_ahead_drop |
| candF_ahead_stronger_9p6 | 14.0 + ahead defaults 9.6/1.22/6.5 | 4/5 | 0.469 | -1.928 | 133.06 | reject_stability_ahead_drop |
| revert_confirm_9p4_speed14p5 | 14.5 + reverted ahead defaults (9.4/1.18/6.2) | 5/5 | 0.511 | -1.789 | 123.08 | monitor_variance |
| candG_dock_ahead_closeblend | 14.5 + docking ahead close-range blend | 4/5 | 0.587 | -1.856 | 157.41 | reject_stability |
| candH_earlyscore2p2 | 14.5 + early-release score threshold 2.2 | 5/5 | 0.593 | -1.966 | 134.13 | promote_runtime_default |
| candI_earlyscore2p3 | 14.5 + early-release score threshold 2.3 | 1/2 | 0.482 | -1.961 | 126.06 | reject_stability |

Key takeaways:
- `baseline_r4e_tuned` remains the kept baseline for now.
- Several candidates reduce path mean, but all tested candidates degrade ahead metrics or stability.
- `candD_speed13p8` keeps `5/5` and lowers path mean, but docking-ahead ratio drops from baseline (`0.659 -> 0.555`), so not promoted.
- `candE_speed14p2` shows instability (one long-tail fail run).
- `candF_ahead_stronger_9p6` lowers path mean but regresses docking-ahead metrics and drops to `4/5`, so reverted.
- Revert-confirm batch (`9.4/1.18/6.2`) returns to `5/5`, but docking-ahead metrics are still below historical baseline and need further tuning.
- `candG_dock_ahead_closeblend` improves docking-ahead mean versus revert-confirm, but drops to `4/5` and increases path/time, so not kept.
- `candH_earlyscore2p2` keeps `5/5`, improves docking-ahead mean (`0.511 -> 0.593`) versus revert-confirm, and keeps path below historical baseline; promoted as runtime default.
- `candI_earlyscore2p3` fails stability in sanity (`1/2`), so not kept.

Code state after sweep:
- Reverted experimental changes that degraded metrics (early-release defaults, lat gate 1.3, corridor catchup 0.16).
- Kept prior stable improvements only (ahead-band first-entry + front-loss corridor release).
