# Same-direction guard batch (5 runs)

- config: `CARRIER_SAME_DIRECTION_GUARD_ENABLED=true`, reverse=0.4, release=0.3, min_forward=0.8, max_lateral=3.0, force=2.0
- v1 pass: 4/5
- loose pass: 4/5
- opp_first20 mean: 0.319
- carrier_r_max mean: 97.58 m

| run | v1 | loose | opp_first20 | carrier_r_max | traj | signed |
|---|---:|---:|---:|---:|---|---|
| 20260410_211126_px4_sih | 0 | 0 | 0.317 | 97.25 | /home/hw/easydocking/results/20260410_211126_px4_sih/trajectory_xy_full.png | /home/hw/easydocking/results/20260410_211126_px4_sih/carrier_front_back_signed_distance.png |
| 20260410_211359_px4_sih | 1 | 1 | 0.315 | 98.22 | /home/hw/easydocking/results/20260410_211359_px4_sih/trajectory_xy_full.png | /home/hw/easydocking/results/20260410_211359_px4_sih/carrier_front_back_signed_distance.png |
| 20260410_211552_px4_sih | 1 | 1 | 0.323 | 97.57 | /home/hw/easydocking/results/20260410_211552_px4_sih/trajectory_xy_full.png | /home/hw/easydocking/results/20260410_211552_px4_sih/carrier_front_back_signed_distance.png |
| 20260410_211748_px4_sih | 1 | 1 | 0.320 | 97.69 | /home/hw/easydocking/results/20260410_211748_px4_sih/trajectory_xy_full.png | /home/hw/easydocking/results/20260410_211748_px4_sih/carrier_front_back_signed_distance.png |
| 20260410_211944_px4_sih | 1 | 1 | 0.322 | 97.16 | /home/hw/easydocking/results/20260410_211944_px4_sih/trajectory_xy_full.png | /home/hw/easydocking/results/20260410_211944_px4_sih/carrier_front_back_signed_distance.png |
