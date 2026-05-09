# RViz silent smoke check

Command:
`START_RVIZ=true RVIZ_SILENT=true AUTO_START_REAR_ENTRY_PREDICTION_PRIMARY_GATE=true CARRIER_TRACKING_SPEED_LIMIT=14.5 ./scripts/run_px4_sih_docking_experiment.sh`

Run:
- `results/20260415_114837_px4_sih`

Validation:
- `LAUNCH_START_RVIZ=false` confirmed in run log output.
- `ros2 launch ... start_rviz:=false` confirmed while `START_RVIZ=true` timing profile is active.
- No `rviz2` process launched during the run.

Outcome (this smoke run):
- `final_pass=1`
- `first_tracking_t_sec=69.28`
- `first_docking_t_sec=83.74`
- `first_completed_t_sec=92.84`
- `track_ahead_ratio_positive=0.847`, `track_ahead_min_m=-0.644`
- `docking_ahead_ratio_positive=0.604`, `docking_ahead_min_m=-0.644`
