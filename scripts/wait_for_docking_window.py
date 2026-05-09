#!/usr/bin/env python3

import math
import sys
import time
from collections import deque
from typing import Optional

import rclpy
from easydocking_msgs.msg import DockingCommand
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import AirspeedValidated, TecsStatus
except ImportError:  # pragma: no cover
    AirspeedValidated = None
    TecsStatus = None


class DockingWindowStarter(Node):
    def __init__(self) -> None:
        super().__init__("docking_window_starter")
        self.declare_parameter("alignment_min", 0.65)
        self.declare_parameter("tca_min_sec", 4.6)
        self.declare_parameter("tca_max_sec", 5.4)
        self.declare_parameter("relative_z_min_m", 20.0)
        self.declare_parameter("relative_distance_min_m", 35.0)
        self.declare_parameter("relative_distance_max_m", 95.0)
        self.declare_parameter("relative_speed_min_mps", 8.0)
        self.declare_parameter("hold_count", 4)
        self.declare_parameter("timeout_sec", 60.0)
        self.declare_parameter("odom_ready_timeout_sec", 25.0)
        self.declare_parameter("min_wait_sec", 0.0)
        self.declare_parameter("publish_repeats", 2)
        self.declare_parameter("fallback_immediate", True)
        self.declare_parameter("release_on_orbit_completion", False)
        self.declare_parameter("release_after_orbit_delay_sec", 0.0)
        self.declare_parameter("prefer_rear_entry", True)
        self.declare_parameter("rear_entry_distance_min_m", 145.0)
        self.declare_parameter("rear_entry_distance_max_m", 185.0)
        self.declare_parameter("rear_entry_min_carrier_ahead_m", 5.0)
        self.declare_parameter("rear_entry_require_carrier_behind", False)
        self.declare_parameter("rear_entry_max_carrier_ahead_m", -5.0)
        self.declare_parameter("rear_entry_max_toward_speed_mps", 9.0)
        self.declare_parameter("rear_entry_max_toward_ratio", 0.62)
        self.declare_parameter("rear_entry_use_prediction_gate", True)
        self.declare_parameter("rear_entry_prediction_primary_gate", False)
        self.declare_parameter("rear_entry_prediction_horizon_sec", 6.0)
        self.declare_parameter("rear_entry_prediction_carrier_speed_mps", 9.5)
        self.declare_parameter("rear_entry_prediction_tangent_weight", 0.65)
        self.declare_parameter("rear_entry_prediction_lateral_max_m", 20.0)
        self.declare_parameter("rear_entry_prediction_along_min_m", 10.0)
        self.declare_parameter("rear_entry_prediction_along_max_m", 70.0)
        self.declare_parameter("rear_entry_prediction_distance_min_m", 60.0)
        self.declare_parameter("rear_entry_prediction_distance_max_m", 110.0)
        self.declare_parameter("rear_entry_prediction_score_threshold", 2.6)
        self.declare_parameter("rear_entry_prediction_score_along_target_m", 24.0)
        self.declare_parameter("rear_entry_prediction_score_along_weight", 1.0)
        self.declare_parameter("rear_entry_prediction_score_lateral_weight", 1.0)
        self.declare_parameter("rear_entry_prediction_score_min_rel_z_target_m", 0.0)
        self.declare_parameter("rear_entry_prediction_score_min_rel_z_scale_m", 1.0)
        self.declare_parameter("rear_entry_prediction_score_min_rel_z_weight", 0.0)
        self.declare_parameter("rear_entry_prediction_local_min_samples", 1)
        self.declare_parameter("rear_entry_prediction_secondary_gate_enabled", True)
        self.declare_parameter("rear_entry_prediction_secondary_hold_count", 12)
        self.declare_parameter("rear_entry_prediction_secondary_score_threshold", 3.2)
        self.declare_parameter("rear_entry_prediction_secondary_lateral_max_m", 24.0)
        self.declare_parameter("rear_entry_prediction_secondary_along_margin_m", 8.0)
        self.declare_parameter("rear_entry_prediction_secondary_distance_min_m", 56.0)
        self.declare_parameter("rear_entry_prediction_secondary_distance_max_m", 125.0)
        self.declare_parameter("rear_entry_prediction_secondary_require_phase_gate", True)
        self.declare_parameter("rear_entry_prediction_secondary_min_after_orbit_sec", 30.0)
        self.declare_parameter("rear_entry_use_global_intercept_gate", True)
        self.declare_parameter("rear_entry_global_intercept_horizon_min_sec", 1.5)
        self.declare_parameter("rear_entry_global_intercept_horizon_max_sec", 9.0)
        self.declare_parameter("rear_entry_global_intercept_horizon_step_sec", 0.5)
        self.declare_parameter("rear_entry_global_intercept_lead_distance_min_m", 8.0)
        self.declare_parameter("rear_entry_global_intercept_lead_distance_max_m", 22.0)
        self.declare_parameter("rear_entry_global_intercept_route_min_forward_cos", 0.45)
        self.declare_parameter("rear_entry_global_intercept_path_weight", 1.0)
        self.declare_parameter("rear_entry_global_intercept_alignment_weight", 10.0)
        self.declare_parameter("rear_entry_global_intercept_time_weight", 0.35)
        self.declare_parameter("rear_entry_global_intercept_max_accel_mps2", 2.5)
        self.declare_parameter("rear_entry_require_global_primary_gate", False)
        self.declare_parameter("rear_entry_global_intercept_primary_score_threshold", 5.7)
        self.declare_parameter("rear_entry_require_non_diverging", True)
        self.declare_parameter("rear_entry_tca_min_sec", 0.8)
        self.declare_parameter("rear_entry_tca_max_sec", 12.0)
        self.declare_parameter("rear_entry_require_orbit_phase_gate", True)
        self.declare_parameter("rear_entry_orbit_phase_center_deg", -95.0)
        self.declare_parameter("rear_entry_orbit_phase_window_deg", 30.0)
        self.declare_parameter("rear_entry_min_mini_vx_mps", 2.0)
        self.declare_parameter("rear_entry_allow_orbit_progress_release", True)
        self.declare_parameter("rear_entry_min_orbit_progress_ratio", 0.92)
        self.declare_parameter("rear_entry_enable_energy_timing_gate", True)
        self.declare_parameter("rear_entry_energy_min_after_orbit_sec", 26.0)
        self.declare_parameter("rear_entry_energy_max_after_orbit_sec", 40.0)
        self.declare_parameter("rear_entry_energy_allow_max_relaxation", True)
        self.declare_parameter("rear_entry_energy_early_release_enabled", True)
        self.declare_parameter("rear_entry_energy_early_release_min_after_orbit_sec", 12.0)
        self.declare_parameter("rear_entry_energy_early_release_hold_count", 4)
        self.declare_parameter("rear_entry_energy_early_release_score_threshold", 2.2)
        self.declare_parameter("rear_entry_energy_early_release_lateral_max_m", 22.0)
        self.declare_parameter("rear_entry_energy_early_release_along_margin_m", 6.0)
        self.declare_parameter("rear_entry_smoke_early_reject_tca_max_sec", 0.0)
        self.declare_parameter("rear_entry_smoke_early_reject_speed_min_mps", 0.0)
        self.declare_parameter("rear_entry_smoke_early_reject_pred_lat_abs_min_m", 0.0)
        self.declare_parameter("rear_entry_smoke_reject_prediction_score_min", 0.0)
        self.declare_parameter("rear_entry_smoke_reject_diag_margin_max_mps", 0.0)
        self.declare_parameter("require_orbit_completion_before_start", True)
        self.declare_parameter("orbit_center_x", 10.0)
        self.declare_parameter("orbit_center_y", -6.0)
        self.declare_parameter("orbit_radius", 80.0)
        self.declare_parameter("orbit_gate_ready_altitude", 28.0)
        self.declare_parameter("orbit_gate_ready_radius_tolerance", 30.0)
        self.declare_parameter("orbit_gate_min_valid_samples", 8)
        self.declare_parameter("orbit_gate_required_laps", 1.0)
        self.declare_parameter("orbit_gate_min_accumulated_angle_deg", 330.0)
        self.declare_parameter("orbit_gate_return_tolerance_deg", 45.0)
        self.declare_parameter("carrier_prehold_required", False)
        self.declare_parameter("carrier_prehold_min_altitude_m", 0.0)
        self.declare_parameter("carrier_prehold_max_abs_vz_mps", 2.5)
        self.declare_parameter("carrier_prehold_min_samples", 1)
        self.declare_parameter("enable_geometry_cluster_gate", False)
        self.declare_parameter("geometry_cluster_score_threshold", 1.0)
        self.declare_parameter("geometry_cluster_min_rel_z", 33.15)
        self.declare_parameter("geometry_cluster_max_rel_z", 33.40)
        self.declare_parameter("geometry_cluster_a_center", [-4.166384, 105.1457845, -6.485333, -10.2991625])
        self.declare_parameter("geometry_cluster_a_spread", [1.6, 1.2, 0.25, 0.25])
        self.declare_parameter("geometry_cluster_b_center", [-5.395847, 102.9623125, -6.2016955, -10.510685])
        self.declare_parameter("geometry_cluster_b_spread", [0.9, 1.2, 0.22, 0.18])
        self.declare_parameter("require_mini_energy_healthy", False)
        self.declare_parameter("mini_px4_namespace", "/px4_2")
        self.declare_parameter("health_min_true_airspeed_mps", 7.0)
        self.declare_parameter("health_max_underspeed_ratio", 0.35)
        self.declare_parameter("health_min_samples", 5)

        self.alignment_min = float(self.get_parameter("alignment_min").value)
        self.tca_min_sec = float(self.get_parameter("tca_min_sec").value)
        self.tca_max_sec = float(self.get_parameter("tca_max_sec").value)
        self.relative_z_min_m = float(self.get_parameter("relative_z_min_m").value)
        self.relative_distance_min_m = float(self.get_parameter("relative_distance_min_m").value)
        self.relative_distance_max_m = float(self.get_parameter("relative_distance_max_m").value)
        self.relative_speed_min_mps = float(self.get_parameter("relative_speed_min_mps").value)
        self.hold_count_required = int(self.get_parameter("hold_count").value)
        self.timeout_sec = float(self.get_parameter("timeout_sec").value)
        self.odom_ready_timeout_sec = max(
            0.0, float(self.get_parameter("odom_ready_timeout_sec").value)
        )
        self.min_wait_sec = max(0.0, float(self.get_parameter("min_wait_sec").value))
        self.publish_repeats = int(self.get_parameter("publish_repeats").value)
        self.fallback_immediate = bool(self.get_parameter("fallback_immediate").value)
        self.release_on_orbit_completion = bool(
            self.get_parameter("release_on_orbit_completion").value
        )
        self.release_after_orbit_delay_sec = max(
            0.0, float(self.get_parameter("release_after_orbit_delay_sec").value)
        )
        self.prefer_rear_entry = bool(self.get_parameter("prefer_rear_entry").value)
        self.rear_entry_distance_min_m = float(
            self.get_parameter("rear_entry_distance_min_m").value
        )
        self.rear_entry_distance_max_m = float(
            self.get_parameter("rear_entry_distance_max_m").value
        )
        self.rear_entry_min_carrier_ahead_m = float(
            self.get_parameter("rear_entry_min_carrier_ahead_m").value
        )
        self.rear_entry_require_carrier_behind = bool(
            self.get_parameter("rear_entry_require_carrier_behind").value
        )
        self.rear_entry_max_carrier_ahead_m = float(
            self.get_parameter("rear_entry_max_carrier_ahead_m").value
        )
        self.rear_entry_max_toward_speed_mps = float(
            self.get_parameter("rear_entry_max_toward_speed_mps").value
        )
        self.rear_entry_max_toward_ratio = float(
            self.get_parameter("rear_entry_max_toward_ratio").value
        )
        self.rear_entry_use_prediction_gate = bool(
            self.get_parameter("rear_entry_use_prediction_gate").value
        )
        self.rear_entry_prediction_primary_gate = bool(
            self.get_parameter("rear_entry_prediction_primary_gate").value
        )
        self.rear_entry_prediction_horizon_sec = max(
            0.0, float(self.get_parameter("rear_entry_prediction_horizon_sec").value)
        )
        self.rear_entry_prediction_carrier_speed_mps = max(
            0.0, float(self.get_parameter("rear_entry_prediction_carrier_speed_mps").value)
        )
        self.rear_entry_prediction_tangent_weight = max(
            0.0,
            min(1.0, float(self.get_parameter("rear_entry_prediction_tangent_weight").value)),
        )
        self.rear_entry_prediction_lateral_max_m = max(
            0.0, float(self.get_parameter("rear_entry_prediction_lateral_max_m").value)
        )
        self.rear_entry_prediction_along_min_m = float(
            self.get_parameter("rear_entry_prediction_along_min_m").value
        )
        self.rear_entry_prediction_along_max_m = float(
            self.get_parameter("rear_entry_prediction_along_max_m").value
        )
        self.rear_entry_prediction_distance_min_m = float(
            self.get_parameter("rear_entry_prediction_distance_min_m").value
        )
        self.rear_entry_prediction_distance_max_m = float(
            self.get_parameter("rear_entry_prediction_distance_max_m").value
        )
        self.rear_entry_prediction_score_threshold = max(
            0.0, float(self.get_parameter("rear_entry_prediction_score_threshold").value)
        )
        self.rear_entry_prediction_score_along_target_m = float(
            self.get_parameter("rear_entry_prediction_score_along_target_m").value
        )
        self.rear_entry_prediction_score_along_weight = max(
            0.0, float(self.get_parameter("rear_entry_prediction_score_along_weight").value)
        )
        self.rear_entry_prediction_score_lateral_weight = max(
            0.0, float(self.get_parameter("rear_entry_prediction_score_lateral_weight").value)
        )
        self.rear_entry_prediction_score_min_rel_z_target_m = float(
            self.get_parameter("rear_entry_prediction_score_min_rel_z_target_m").value
        )
        self.rear_entry_prediction_score_min_rel_z_scale_m = max(
            1e-6,
            float(self.get_parameter("rear_entry_prediction_score_min_rel_z_scale_m").value),
        )
        self.rear_entry_prediction_score_min_rel_z_weight = max(
            0.0, float(self.get_parameter("rear_entry_prediction_score_min_rel_z_weight").value)
        )
        self.rear_entry_prediction_local_min_samples = max(
            1, int(self.get_parameter("rear_entry_prediction_local_min_samples").value)
        )
        self.rear_entry_prediction_secondary_gate_enabled = bool(
            self.get_parameter("rear_entry_prediction_secondary_gate_enabled").value
        )
        self.rear_entry_prediction_secondary_hold_count = max(
            1, int(self.get_parameter("rear_entry_prediction_secondary_hold_count").value)
        )
        self.rear_entry_prediction_secondary_score_threshold = max(
            0.0, float(self.get_parameter("rear_entry_prediction_secondary_score_threshold").value)
        )
        self.rear_entry_prediction_secondary_lateral_max_m = max(
            0.0, float(self.get_parameter("rear_entry_prediction_secondary_lateral_max_m").value)
        )
        self.rear_entry_prediction_secondary_along_margin_m = max(
            0.0, float(self.get_parameter("rear_entry_prediction_secondary_along_margin_m").value)
        )
        self.rear_entry_prediction_secondary_distance_min_m = float(
            self.get_parameter("rear_entry_prediction_secondary_distance_min_m").value
        )
        self.rear_entry_prediction_secondary_distance_max_m = float(
            self.get_parameter("rear_entry_prediction_secondary_distance_max_m").value
        )
        self.rear_entry_prediction_secondary_require_phase_gate = bool(
            self.get_parameter("rear_entry_prediction_secondary_require_phase_gate").value
        )
        self.rear_entry_prediction_secondary_min_after_orbit_sec = max(
            0.0, float(self.get_parameter("rear_entry_prediction_secondary_min_after_orbit_sec").value)
        )
        self.rear_entry_use_global_intercept_gate = bool(
            self.get_parameter("rear_entry_use_global_intercept_gate").value
        )
        self.rear_entry_global_intercept_horizon_min_sec = max(
            0.2, float(self.get_parameter("rear_entry_global_intercept_horizon_min_sec").value)
        )
        self.rear_entry_global_intercept_horizon_max_sec = max(
            self.rear_entry_global_intercept_horizon_min_sec,
            float(self.get_parameter("rear_entry_global_intercept_horizon_max_sec").value),
        )
        self.rear_entry_global_intercept_horizon_step_sec = max(
            0.1, float(self.get_parameter("rear_entry_global_intercept_horizon_step_sec").value)
        )
        self.rear_entry_global_intercept_lead_distance_min_m = max(
            0.0, float(self.get_parameter("rear_entry_global_intercept_lead_distance_min_m").value)
        )
        self.rear_entry_global_intercept_lead_distance_max_m = max(
            self.rear_entry_global_intercept_lead_distance_min_m,
            float(self.get_parameter("rear_entry_global_intercept_lead_distance_max_m").value),
        )
        self.rear_entry_global_intercept_route_min_forward_cos = max(
            -1.0,
            min(1.0, float(self.get_parameter("rear_entry_global_intercept_route_min_forward_cos").value)),
        )
        self.rear_entry_global_intercept_path_weight = max(
            0.0, float(self.get_parameter("rear_entry_global_intercept_path_weight").value)
        )
        self.rear_entry_global_intercept_alignment_weight = max(
            0.0, float(self.get_parameter("rear_entry_global_intercept_alignment_weight").value)
        )
        self.rear_entry_global_intercept_time_weight = max(
            0.0, float(self.get_parameter("rear_entry_global_intercept_time_weight").value)
        )
        self.rear_entry_global_intercept_max_accel_mps2 = max(
            0.0, float(self.get_parameter("rear_entry_global_intercept_max_accel_mps2").value)
        )
        self.rear_entry_require_global_primary_gate = bool(
            self.get_parameter("rear_entry_require_global_primary_gate").value
        )
        self.rear_entry_global_intercept_primary_score_threshold = max(
            0.0,
            float(self.get_parameter("rear_entry_global_intercept_primary_score_threshold").value),
        )
        self.rear_entry_require_non_diverging = bool(
            self.get_parameter("rear_entry_require_non_diverging").value
        )
        self.rear_entry_tca_min_sec = float(
            self.get_parameter("rear_entry_tca_min_sec").value
        )
        self.rear_entry_tca_max_sec = float(
            self.get_parameter("rear_entry_tca_max_sec").value
        )
        self.rear_entry_require_orbit_phase_gate = bool(
            self.get_parameter("rear_entry_require_orbit_phase_gate").value
        )
        self.rear_entry_orbit_phase_center_rad = math.radians(float(
            self.get_parameter("rear_entry_orbit_phase_center_deg").value
        ))
        self.rear_entry_orbit_phase_window_rad = math.radians(max(
            0.0, float(self.get_parameter("rear_entry_orbit_phase_window_deg").value)
        ))
        self.rear_entry_min_mini_vx_mps = float(
            self.get_parameter("rear_entry_min_mini_vx_mps").value
        )
        self.rear_entry_allow_orbit_progress_release = bool(
            self.get_parameter("rear_entry_allow_orbit_progress_release").value
        )
        self.rear_entry_min_orbit_progress_ratio = max(
            0.0,
            min(1.0, float(self.get_parameter("rear_entry_min_orbit_progress_ratio").value)),
        )
        self.rear_entry_enable_energy_timing_gate = bool(
            self.get_parameter("rear_entry_enable_energy_timing_gate").value
        )
        self.rear_entry_energy_min_after_orbit_sec = max(
            0.0, float(self.get_parameter("rear_entry_energy_min_after_orbit_sec").value)
        )
        self.rear_entry_energy_max_after_orbit_sec = float(
            self.get_parameter("rear_entry_energy_max_after_orbit_sec").value
        )
        self.rear_entry_energy_allow_max_relaxation = bool(
            self.get_parameter("rear_entry_energy_allow_max_relaxation").value
        )
        self.rear_entry_energy_early_release_enabled = bool(
            self.get_parameter("rear_entry_energy_early_release_enabled").value
        )
        self.rear_entry_energy_early_release_min_after_orbit_sec = max(
            0.0, float(self.get_parameter("rear_entry_energy_early_release_min_after_orbit_sec").value)
        )
        self.rear_entry_energy_early_release_hold_count = max(
            1, int(self.get_parameter("rear_entry_energy_early_release_hold_count").value)
        )
        self.rear_entry_energy_early_release_score_threshold = max(
            0.0, float(self.get_parameter("rear_entry_energy_early_release_score_threshold").value)
        )
        self.rear_entry_energy_early_release_lateral_max_m = max(
            0.0, float(self.get_parameter("rear_entry_energy_early_release_lateral_max_m").value)
        )
        self.rear_entry_energy_early_release_along_margin_m = max(
            0.0, float(self.get_parameter("rear_entry_energy_early_release_along_margin_m").value)
        )
        self.rear_entry_smoke_early_reject_tca_max_sec = max(
            0.0, float(self.get_parameter("rear_entry_smoke_early_reject_tca_max_sec").value)
        )
        self.rear_entry_smoke_early_reject_speed_min_mps = max(
            0.0, float(self.get_parameter("rear_entry_smoke_early_reject_speed_min_mps").value)
        )
        self.rear_entry_smoke_early_reject_pred_lat_abs_min_m = max(
            0.0,
            float(self.get_parameter("rear_entry_smoke_early_reject_pred_lat_abs_min_m").value),
        )
        self.rear_entry_smoke_reject_prediction_score_min = max(
            0.0,
            float(self.get_parameter("rear_entry_smoke_reject_prediction_score_min").value),
        )
        self.rear_entry_smoke_reject_diag_margin_max_mps = max(
            0.0,
            float(self.get_parameter("rear_entry_smoke_reject_diag_margin_max_mps").value),
        )
        self.require_orbit_completion_before_start = bool(
            self.get_parameter("require_orbit_completion_before_start").value
        )
        self.orbit_center_x = float(self.get_parameter("orbit_center_x").value)
        self.orbit_center_y = float(self.get_parameter("orbit_center_y").value)
        self.orbit_radius = float(self.get_parameter("orbit_radius").value)
        self.orbit_gate_ready_altitude = float(
            self.get_parameter("orbit_gate_ready_altitude").value
        )
        self.orbit_gate_ready_radius_tolerance = float(
            self.get_parameter("orbit_gate_ready_radius_tolerance").value
        )
        self.orbit_gate_min_valid_samples = int(
            self.get_parameter("orbit_gate_min_valid_samples").value
        )
        self.orbit_gate_required_laps = float(
            self.get_parameter("orbit_gate_required_laps").value
        )
        self.orbit_gate_min_accumulated_angle_rad = math.radians(float(
            self.get_parameter("orbit_gate_min_accumulated_angle_deg").value
        ))
        self.orbit_gate_return_tolerance_rad = math.radians(float(
            self.get_parameter("orbit_gate_return_tolerance_deg").value
        ))
        self.carrier_prehold_required = bool(
            self.get_parameter("carrier_prehold_required").value
        )
        self.carrier_prehold_min_altitude_m = float(
            self.get_parameter("carrier_prehold_min_altitude_m").value
        )
        self.carrier_prehold_max_abs_vz_mps = max(
            0.0, float(self.get_parameter("carrier_prehold_max_abs_vz_mps").value)
        )
        self.carrier_prehold_min_samples = max(
            1, int(self.get_parameter("carrier_prehold_min_samples").value)
        )
        self.enable_geometry_cluster_gate = bool(
            self.get_parameter("enable_geometry_cluster_gate").value
        )
        self.geometry_cluster_score_threshold = float(
            self.get_parameter("geometry_cluster_score_threshold").value
        )
        self.geometry_cluster_min_rel_z = float(
            self.get_parameter("geometry_cluster_min_rel_z").value
        )
        self.geometry_cluster_max_rel_z = float(
            self.get_parameter("geometry_cluster_max_rel_z").value
        )
        self.geometry_cluster_a_center = [
            float(v) for v in self.get_parameter("geometry_cluster_a_center").value
        ]
        self.geometry_cluster_a_spread = [
            float(v) for v in self.get_parameter("geometry_cluster_a_spread").value
        ]
        self.geometry_cluster_b_center = [
            float(v) for v in self.get_parameter("geometry_cluster_b_center").value
        ]
        self.geometry_cluster_b_spread = [
            float(v) for v in self.get_parameter("geometry_cluster_b_spread").value
        ]
        self.require_mini_energy_healthy = bool(
            self.get_parameter("require_mini_energy_healthy").value
        )
        self.mini_px4_namespace = str(self.get_parameter("mini_px4_namespace").value)
        self.health_min_true_airspeed_mps = float(
            self.get_parameter("health_min_true_airspeed_mps").value
        )
        self.health_max_underspeed_ratio = float(
            self.get_parameter("health_max_underspeed_ratio").value
        )
        self.health_min_samples = int(self.get_parameter("health_min_samples").value)

        self.carrier_odom: Optional[Odometry] = None
        self.mini_odom: Optional[Odometry] = None
        self.hold_count = 0
        self.command_sent = False
        self.start_time = self.get_clock().now()
        self.orbit_gate_ready_samples = 0
        self.orbit_gate_tracking_active = not self.require_orbit_completion_before_start
        self.orbit_gate_completed = not self.require_orbit_completion_before_start
        self.orbit_gate_initial_angle: Optional[float] = None
        self.orbit_gate_previous_angle: Optional[float] = None
        self.orbit_gate_accumulated_angle = 0.0
        self.orbit_gate_last_reported_quarter = -1
        self.orbit_gate_completed_elapsed_sec: Optional[float] = 0.0 if self.orbit_gate_completed else None
        self.carrier_prehold_good_samples = 0
        self.carrier_prehold_announced = None
        self.rear_entry_energy_max_relaxed = False
        self.mini_true_airspeed_mps = math.nan
        self.mini_underspeed_ratio = math.nan
        self.mini_health_good_samples = 0
        self.mini_health_announced = None
        self.prediction_score_history: deque[float] = deque(
            maxlen=self.rear_entry_prediction_local_min_samples
        )
        self.prediction_secondary_hold_streak = 0
        self.energy_early_release_hold_streak = 0

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.command_pub = self.create_publisher(DockingCommand, "/docking/command", 10)
        self.command_latched_pub = self.create_publisher(
            DockingCommand, "/docking/command_latched", latched_qos
        )
        self.create_subscription(Odometry, "/carrier/odom", self._carrier_cb, 10)
        self.create_subscription(Odometry, "/mini/odom", self._mini_cb, 10)
        if self.require_mini_energy_healthy and AirspeedValidated is not None:
            self._subscribe_mini_airspeed_topics(px4_qos)
        if self.require_mini_energy_healthy and TecsStatus is not None:
            self._subscribe_mini_tecs_topics(px4_qos)
        self.timer = self.create_timer(0.05, self._tick)

    def _carrier_cb(self, msg: Odometry) -> None:
        self.carrier_odom = msg

    def _mini_cb(self, msg: Odometry) -> None:
        self.mini_odom = msg

    def _mini_airspeed_cb(self, msg: AirspeedValidated) -> None:
        self.mini_true_airspeed_mps = float(msg.true_airspeed_m_s)

    def _mini_tecs_cb(self, msg: TecsStatus) -> None:
        self.mini_underspeed_ratio = float(msg.underspeed_ratio)

    def _subscribe_mini_airspeed_topics(self, px4_qos: QoSProfile) -> None:
        topics = (
            "airspeed_validated",
            "airspeed_validated_v1",
            "airspeed_validated_v2",
            "airspeed_validated_v3",
        )
        for topic in topics:
            self.create_subscription(
                AirspeedValidated,
                f"{self.mini_px4_namespace}/fmu/out/{topic}",
                self._mini_airspeed_cb,
                px4_qos,
            )

    def _subscribe_mini_tecs_topics(self, px4_qos: QoSProfile) -> None:
        topics = (
            "tecs_status",
            "tecs_status_v1",
            "tecs_status_v2",
            "tecs_status_v3",
        )
        for topic in topics:
            self.create_subscription(
                TecsStatus,
                f"{self.mini_px4_namespace}/fmu/out/{topic}",
                self._mini_tecs_cb,
                px4_qos,
            )

    @staticmethod
    def _reachable_average_speed(
        initial_speed: float,
        speed_limit: float,
        max_accel: float,
        horizon_sec: float,
    ) -> float:
        horizon = max(horizon_sec, 1e-3)
        clamped_initial = max(0.0, min(initial_speed, speed_limit))
        if max_accel <= 0.0 or speed_limit <= 0.0:
            return max(0.0, speed_limit)
        if clamped_initial >= speed_limit - 1e-6:
            return speed_limit

        accel_time = (speed_limit - clamped_initial) / max_accel
        if horizon <= accel_time:
            return clamped_initial + 0.5 * max_accel * horizon

        accel_distance = clamped_initial * accel_time + 0.5 * max_accel * accel_time * accel_time
        cruise_distance = speed_limit * (horizon - accel_time)
        return (accel_distance + cruise_distance) / horizon

    @staticmethod
    def _normalize_xy(x: float, y: float, fallback_x: float, fallback_y: float) -> tuple[float, float]:
        norm = math.hypot(x, y)
        if norm > 1e-6:
            return x / norm, y / norm
        fallback_norm = math.hypot(fallback_x, fallback_y)
        if fallback_norm > 1e-6:
            return fallback_x / fallback_norm, fallback_y / fallback_norm
        return 1.0, 0.0

    def _compute_global_intercept_candidate(
        self,
        rel_distance_xy: float,
        rel_z: float,
    ) -> tuple[Optional[dict], dict]:
        if self.carrier_odom is None or self.mini_odom is None:
            return None, {}

        carrier_pos_x = self.carrier_odom.pose.pose.position.x
        carrier_pos_y = self.carrier_odom.pose.pose.position.y
        carrier_vx = self.carrier_odom.twist.twist.linear.x
        carrier_vy = self.carrier_odom.twist.twist.linear.y

        mini_pos_x = self.mini_odom.pose.pose.position.x
        mini_pos_y = self.mini_odom.pose.pose.position.y
        mini_vx = self.mini_odom.twist.twist.linear.x
        mini_vy = self.mini_odom.twist.twist.linear.y

        mini_speed_xy = math.hypot(mini_vx, mini_vy)
        if mini_speed_xy <= 1e-6:
            return None, {}

        radial_x = mini_pos_x - self.orbit_center_x
        radial_y = mini_pos_y - self.orbit_center_y
        radial_norm = math.hypot(radial_x, radial_y)
        if radial_norm <= 1e-6:
            return None, {}

        phase_now = math.atan2(radial_y, radial_x)
        current_track_x, current_track_y = self._normalize_xy(mini_vx, mini_vy, 1.0, 0.0)
        tangent_ccw_x = -math.sin(phase_now)
        tangent_ccw_y = math.cos(phase_now)
        orbit_direction_sign = 1.0 if (mini_vx * tangent_ccw_x + mini_vy * tangent_ccw_y) >= 0.0 else -1.0
        orbit_speed_xy = mini_speed_xy
        orbit_rate = orbit_direction_sign * orbit_speed_xy / max(self.orbit_radius, 1.0)

        preferred_lead_distance = min(
            self.rear_entry_global_intercept_lead_distance_max_m,
            max(
                self.rear_entry_global_intercept_lead_distance_min_m,
                max(
                    abs(self.rear_entry_max_carrier_ahead_m) if self.rear_entry_require_carrier_behind
                    else self.rear_entry_min_carrier_ahead_m,
                    self.rear_entry_global_intercept_lead_distance_min_m,
                ),
            ),
        )

        best: Optional[dict] = None
        best_relaxed: Optional[dict] = None
        diag = {
            "candidate_count": 0,
            "degenerate_reject_count": 0,
            "route_reject_count": 0,
            "speed_reject_count": 0,
        }
        tau = self.rear_entry_global_intercept_horizon_min_sec
        while tau <= self.rear_entry_global_intercept_horizon_max_sec + 1e-6:
            future_phase = phase_now + orbit_rate * tau
            future_radial_x = math.cos(future_phase)
            future_radial_y = math.sin(future_phase)
            future_track_x, future_track_y = self._normalize_xy(
                -orbit_direction_sign * math.sin(future_phase),
                orbit_direction_sign * math.cos(future_phase),
                current_track_x,
                current_track_y,
            )
            lead_distance = min(
                self.rear_entry_global_intercept_lead_distance_max_m,
                max(
                    self.rear_entry_global_intercept_lead_distance_min_m,
                    preferred_lead_distance +
                    0.45 * (tau - self.rear_entry_global_intercept_horizon_min_sec),
                ),
            )

            future_mini_x = self.orbit_center_x + future_radial_x * self.orbit_radius
            future_mini_y = self.orbit_center_y + future_radial_y * self.orbit_radius
            lead_sign = -1.0 if self.rear_entry_require_carrier_behind else 1.0
            candidate_x = future_mini_x + lead_sign * future_track_x * lead_distance
            candidate_y = future_mini_y + lead_sign * future_track_y * lead_distance

            route_x = candidate_x - carrier_pos_x
            route_y = candidate_y - carrier_pos_y
            route_distance = math.hypot(route_x, route_y)
            if route_distance <= 1e-6:
                diag["degenerate_reject_count"] += 1
                tau += self.rear_entry_global_intercept_horizon_step_sec
                continue

            diag["candidate_count"] += 1
            route_dir_x = route_x / route_distance
            route_dir_y = route_y / route_distance
            route_forward_cos = route_dir_x * current_track_x + route_dir_y * current_track_y
            current_route_speed = max(0.0, carrier_vx * route_dir_x + carrier_vy * route_dir_y)
            reachable_avg_speed = self._reachable_average_speed(
                current_route_speed,
                self.rear_entry_prediction_carrier_speed_mps,
                self.rear_entry_global_intercept_max_accel_mps2,
                tau,
            )
            required_avg_speed = route_distance / max(tau, 1e-3)
            speed_margin = reachable_avg_speed - required_avg_speed
            reachable_distance = min(route_distance, max(0.0, reachable_avg_speed * tau))
            carrier_pred_x = carrier_pos_x + route_dir_x * reachable_distance
            carrier_pred_y = carrier_pos_y + route_dir_y * reachable_distance
            prediction_dx = future_mini_x - carrier_pred_x
            prediction_dy = future_mini_y - carrier_pred_y
            future_lat_x = -future_track_y
            future_lat_y = future_track_x
            prediction_along_m = prediction_dx * future_track_x + prediction_dy * future_track_y
            prediction_lateral_m = prediction_dx * future_lat_x + prediction_dy * future_lat_y
            along_target_m = (
                self.rear_entry_prediction_score_along_target_m
                if self.rear_entry_require_carrier_behind
                else -self.rear_entry_prediction_score_along_target_m
            )
            prediction_along_min_m = (
                self.rear_entry_prediction_along_min_m
                if self.rear_entry_require_carrier_behind
                else -self.rear_entry_prediction_along_max_m
            )
            prediction_along_max_m = (
                self.rear_entry_prediction_along_max_m
                if self.rear_entry_require_carrier_behind
                else -self.rear_entry_prediction_along_min_m
            )
            along_scale = max(
                abs(prediction_along_max_m - along_target_m),
                abs(along_target_m - prediction_along_min_m),
                1e-6,
            )
            lateral_scale = max(self.rear_entry_prediction_lateral_max_m, 1e-6)
            geometry_score = (
                self.rear_entry_prediction_score_lateral_weight *
                (abs(prediction_lateral_m) / lateral_scale) +
                self.rear_entry_prediction_score_along_weight *
                (abs(prediction_along_m - along_target_m) / along_scale)
            )
            geometry_score += self._prediction_min_rel_z_penalty(rel_z)
            path_ratio = route_distance / max(rel_distance_xy, 1.0)
            time_ratio = tau / max(self.rear_entry_global_intercept_horizon_max_sec, 1.0)
            speed_penalty = max(
                0.0,
                -speed_margin / max(self.rear_entry_prediction_carrier_speed_mps, 1.0),
            )
            score = (
                geometry_score +
                self.rear_entry_global_intercept_path_weight * path_ratio +
                self.rear_entry_global_intercept_alignment_weight * max(0.0, 1.0 - route_forward_cos) +
                self.rear_entry_global_intercept_time_weight * time_ratio +
                speed_penalty
            )
            route_ok = route_forward_cos >= self.rear_entry_global_intercept_route_min_forward_cos
            speed_ok = (
                speed_margin >=
                -0.35 * max(self.rear_entry_prediction_carrier_speed_mps, 1.0)
            )

            candidate_info = {
                "tau": tau,
                "future_mini_x": future_mini_x,
                "future_mini_y": future_mini_y,
                "candidate_x": candidate_x,
                "candidate_y": candidate_y,
                "carrier_pred_x": carrier_pred_x,
                "carrier_pred_y": carrier_pred_y,
                "track_x": future_track_x,
                "track_y": future_track_y,
                "route_distance": route_distance,
                "route_forward_cos": route_forward_cos,
                "reachable_avg_speed": reachable_avg_speed,
                "required_avg_speed": required_avg_speed,
                "speed_margin": speed_margin,
                "lead_distance": lead_distance,
                "reachable_distance": reachable_distance,
                "prediction_along_m": prediction_along_m,
                "prediction_lateral_m": prediction_lateral_m,
                "geometry_score": geometry_score,
                "path_ratio": path_ratio,
                "time_ratio": time_ratio,
                "score": score,
                "route_ok": route_ok,
                "speed_ok": speed_ok,
            }
            if best_relaxed is None or candidate_info["score"] < best_relaxed["score"]:
                best_relaxed = candidate_info

            if not route_ok:
                diag["route_reject_count"] += 1
                tau += self.rear_entry_global_intercept_horizon_step_sec
                continue
            if not speed_ok:
                diag["speed_reject_count"] += 1
                tau += self.rear_entry_global_intercept_horizon_step_sec
                continue
            if best is None or candidate_info["score"] < best["score"]:
                best = candidate_info

            tau += self.rear_entry_global_intercept_horizon_step_sec

        if best_relaxed is not None:
            diag.update({
                "relaxed_tau": best_relaxed["tau"],
                "relaxed_route_distance_m": best_relaxed["route_distance"],
                "relaxed_route_forward_cos": best_relaxed["route_forward_cos"],
                "relaxed_speed_margin_mps": best_relaxed["speed_margin"],
                "relaxed_prediction_along_m": best_relaxed["prediction_along_m"],
                "relaxed_prediction_lateral_m": best_relaxed["prediction_lateral_m"],
                "relaxed_score": best_relaxed["score"],
                "relaxed_path_ratio": best_relaxed["path_ratio"],
                "relaxed_time_ratio": best_relaxed["time_ratio"],
                "relaxed_route_ok": 1.0 if best_relaxed["route_ok"] else 0.0,
                "relaxed_speed_ok": 1.0 if best_relaxed["speed_ok"] else 0.0,
            })

        return best, diag

    def _tick(self) -> None:
        if self.command_sent:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        timeout_elapsed = elapsed
        if (
            self.require_orbit_completion_before_start and
            self.orbit_gate_completed_elapsed_sec is not None
        ):
            timeout_elapsed = elapsed - self.orbit_gate_completed_elapsed_sec
        if self.carrier_odom is None or self.mini_odom is None:
            odom_timeout_sec = self.timeout_sec
            if self.odom_ready_timeout_sec > 0.0:
                odom_timeout_sec = min(odom_timeout_sec, self.odom_ready_timeout_sec)
            if timeout_elapsed > odom_timeout_sec and self.fallback_immediate:
                self.get_logger().warn("Timeout before odom ready, publishing START anyway")
                self._publish_start("timeout_no_odom")
            return
        rel_x = self.mini_odom.pose.pose.position.x - self.carrier_odom.pose.pose.position.x
        rel_y = self.mini_odom.pose.pose.position.y - self.carrier_odom.pose.pose.position.y
        rel_z = self.mini_odom.pose.pose.position.z - self.carrier_odom.pose.pose.position.z
        rel_vx = self.mini_odom.twist.twist.linear.x - self.carrier_odom.twist.twist.linear.x
        rel_vy = self.mini_odom.twist.twist.linear.y - self.carrier_odom.twist.twist.linear.y
        carrier_z = float(self.carrier_odom.pose.pose.position.z)
        carrier_vz = float(self.carrier_odom.twist.twist.linear.z)

        self._update_orbit_gate()
        carrier_prehold_ready = self._carrier_prehold_ready()
        if not carrier_prehold_ready:
            self.hold_count = 0
            if timeout_elapsed > self.timeout_sec:
                orbit_ready_for_fallback = (
                    not self.require_orbit_completion_before_start or
                    self.orbit_gate_completed
                )
                if self.fallback_immediate and orbit_ready_for_fallback:
                    self.get_logger().warn(
                        "Timeout waiting for carrier prehold, publishing START anyway"
                    )
                    self._publish_start("timeout_prehold")
                else:
                    self.get_logger().error("Timeout waiting for carrier prehold")
                    rclpy.shutdown()
                    sys.exit(1)
            return
        if not self._mini_energy_healthy():
            self.hold_count = 0
            if timeout_elapsed > self.timeout_sec:
                orbit_ready_for_fallback = (
                    not self.require_orbit_completion_before_start or
                    self.orbit_gate_completed
                )
                if self.fallback_immediate and orbit_ready_for_fallback:
                    self.get_logger().warn(
                        "Timeout waiting for mini-energy health, publishing START anyway"
                    )
                    self._publish_start("timeout_energy_health")
                else:
                    self.get_logger().error("Timeout waiting for mini-energy health")
                    rclpy.shutdown()
                    sys.exit(1)
            return
        if elapsed < self.min_wait_sec:
            self.hold_count = 0
            return
        rel_distance_xy = math.hypot(rel_x, rel_y)
        rel_speed_xy = math.hypot(rel_vx, rel_vy)
        mini_speed_xy = math.hypot(
            self.mini_odom.twist.twist.linear.x,
            self.mini_odom.twist.twist.linear.y,
        )
        carrier_ahead_along_mini = math.nan
        mini_toward_carrier_speed = math.nan
        mini_toward_carrier_ratio = math.nan
        if rel_distance_xy > 1e-6 and mini_speed_xy > 1e-6:
            mini_dir_x = self.mini_odom.twist.twist.linear.x / mini_speed_xy
            mini_dir_y = self.mini_odom.twist.twist.linear.y / mini_speed_xy
            carrier_ahead_along_mini = (
                (self.carrier_odom.pose.pose.position.x - self.mini_odom.pose.pose.position.x) * mini_dir_x +
                (self.carrier_odom.pose.pose.position.y - self.mini_odom.pose.pose.position.y) * mini_dir_y
            )
            los_to_carrier_x = (
                self.carrier_odom.pose.pose.position.x - self.mini_odom.pose.pose.position.x
            ) / rel_distance_xy
            los_to_carrier_y = (
                self.carrier_odom.pose.pose.position.y - self.mini_odom.pose.pose.position.y
            ) / rel_distance_xy
            mini_toward_carrier_speed = (
                self.mini_odom.twist.twist.linear.x * los_to_carrier_x +
                self.mini_odom.twist.twist.linear.y * los_to_carrier_y
            )
            mini_toward_carrier_ratio = mini_toward_carrier_speed / mini_speed_xy
        if rel_distance_xy < 1e-6 or rel_speed_xy < 1e-6:
            alignment = -1.0
            tca_sec = math.inf
        else:
            alignment = (-rel_x * rel_vx - rel_y * rel_vy) / (rel_distance_xy * rel_speed_xy)
            tca_sec = -(rel_x * rel_vx + rel_y * rel_vy) / max(rel_speed_xy ** 2, 1e-6)

        geometry_cluster_score = min(
            self._geometry_cluster_score(
                (rel_x, rel_y, rel_vx, rel_vy),
                self.geometry_cluster_a_center,
                self.geometry_cluster_a_spread,
            ),
            self._geometry_cluster_score(
                (rel_x, rel_y, rel_vx, rel_vy),
                self.geometry_cluster_b_center,
                self.geometry_cluster_b_spread,
            ),
        )
        geometry_cluster_ok = (
            not self.enable_geometry_cluster_gate or
            (
                geometry_cluster_score <= self.geometry_cluster_score_threshold and
                self.geometry_cluster_min_rel_z <= rel_z <= self.geometry_cluster_max_rel_z
            )
        )

        common_window_ok = (
            rel_z >= self.relative_z_min_m and
            rel_speed_xy >= self.relative_speed_min_mps and
            rel_distance_xy >= self.relative_distance_min_m and
            rel_distance_xy <= self.relative_distance_max_m and
            geometry_cluster_ok
        )
        legacy_window_ok = (
            common_window_ok and
            alignment >= self.alignment_min and
            self.tca_min_sec <= tca_sec <= self.tca_max_sec
        )
        rear_entry_orbit_progress_ready = (
            self.orbit_gate_tracking_active and
            abs(self.orbit_gate_accumulated_angle) >=
            self.orbit_gate_min_accumulated_angle_rad * self.rear_entry_min_orbit_progress_ratio
        )
        rear_entry_elapsed_after_orbit_sec = math.nan
        rear_entry_energy_timing_ok = True
        rear_entry_energy_relaxed = False
        if self.orbit_gate_completed_elapsed_sec is not None:
            rear_entry_elapsed_after_orbit_sec = elapsed - self.orbit_gate_completed_elapsed_sec
        if self.rear_entry_enable_energy_timing_gate:
            if self.orbit_gate_completed_elapsed_sec is None:
                rear_entry_energy_timing_ok = False
            else:
                rear_entry_energy_timing_ok = (
                    rear_entry_elapsed_after_orbit_sec >= self.rear_entry_energy_min_after_orbit_sec
                )
                if self.rear_entry_energy_max_after_orbit_sec > 0.0:
                    if rear_entry_elapsed_after_orbit_sec <= self.rear_entry_energy_max_after_orbit_sec:
                        pass
                    elif (
                        self.rear_entry_energy_allow_max_relaxation and
                        (self.rear_entry_prediction_primary_gate or self.rear_entry_use_prediction_gate)
                    ):
                        rear_entry_energy_relaxed = True
                        if not self.rear_entry_energy_max_relaxed:
                            self.rear_entry_energy_max_relaxed = True
                            self.get_logger().warn(
                                "Energy timing max gate relaxed for prediction-based start "
                                f"elapsed_after_orbit={rear_entry_elapsed_after_orbit_sec:.2f}s "
                                f"max={self.rear_entry_energy_max_after_orbit_sec:.2f}s"
                            )
                    else:
                        rear_entry_energy_timing_ok = False
        mini_orbit_dx = self.mini_odom.pose.pose.position.x - self.orbit_center_x
        mini_orbit_dy = self.mini_odom.pose.pose.position.y - self.orbit_center_y
        mini_orbit_phase = math.atan2(mini_orbit_dy, mini_orbit_dx)
        mini_orbit_phase_error = abs(
            self._normalize_angle(mini_orbit_phase - self.rear_entry_orbit_phase_center_rad)
        )
        rear_entry_phase_ok = (
            (not self.rear_entry_require_orbit_phase_gate) or
            mini_orbit_phase_error <= self.rear_entry_orbit_phase_window_rad
        )
        rear_entry_mini_vx_ok = (
            self.mini_odom.twist.twist.linear.x >= self.rear_entry_min_mini_vx_mps
        )
        prediction_along_m = math.nan
        prediction_lateral_m = math.nan
        prediction_score = math.nan
        prediction_along_min_m = math.nan
        prediction_along_max_m = math.nan
        prediction_score_ok = False
        prediction_distance_ok = (
            self.rear_entry_prediction_distance_min_m <= rel_distance_xy <=
            self.rear_entry_prediction_distance_max_m
        )
        prediction_local_min_ok = False
        prediction_gate_ok = False
        prediction_secondary_score_ok = False
        prediction_secondary_core_ok = False
        prediction_secondary_direction_ok = False
        prediction_secondary_timing_ok = False
        prediction_secondary_gate_ok = False
        prediction_early_score_ok = False
        prediction_early_geom_ok = False
        prediction_early_along_ok = False
        intercept_front_geometry_ok = False
        global_intercept_primary_gate_ok = False
        global_intercept_block_local_release = False
        intercept_release_timing_ok = False
        energy_early_release_candidate = False
        energy_early_release_ok = False
        global_intercept_tau = math.nan
        global_intercept_route_distance_m = math.nan
        global_intercept_route_forward_cos = math.nan
        global_intercept_speed_margin_mps = math.nan
        global_intercept_relaxed_tau = math.nan
        global_intercept_relaxed_route_distance_m = math.nan
        global_intercept_relaxed_route_forward_cos = math.nan
        global_intercept_relaxed_speed_margin_mps = math.nan
        global_intercept_relaxed_prediction_lateral_m = math.nan
        global_intercept_relaxed_score = math.nan
        global_intercept_relaxed_path_ratio = math.nan
        global_intercept_relaxed_route_ok = 0.0
        global_intercept_relaxed_speed_ok = 0.0
        global_intercept_candidate_count = 0.0
        global_intercept_route_reject_count = 0.0
        global_intercept_speed_reject_count = 0.0
        rear_entry_ahead_ok = (
            math.isfinite(carrier_ahead_along_mini) and
            (
                carrier_ahead_along_mini <= self.rear_entry_max_carrier_ahead_m
                if self.rear_entry_require_carrier_behind
                else carrier_ahead_along_mini >= self.rear_entry_min_carrier_ahead_m
            )
        )
        intercept_candidate = None
        if self.rear_entry_use_global_intercept_gate:
            intercept_candidate, intercept_diag = self._compute_global_intercept_candidate(rel_distance_xy, rel_z)
            if intercept_diag:
                global_intercept_relaxed_tau = intercept_diag.get("relaxed_tau", math.nan)
                global_intercept_relaxed_route_distance_m = intercept_diag.get("relaxed_route_distance_m", math.nan)
                global_intercept_relaxed_route_forward_cos = intercept_diag.get("relaxed_route_forward_cos", math.nan)
                global_intercept_relaxed_speed_margin_mps = intercept_diag.get("relaxed_speed_margin_mps", math.nan)
                global_intercept_relaxed_prediction_lateral_m = intercept_diag.get("relaxed_prediction_lateral_m", math.nan)
                global_intercept_relaxed_score = intercept_diag.get("relaxed_score", math.nan)
                global_intercept_relaxed_path_ratio = intercept_diag.get("relaxed_path_ratio", math.nan)
                global_intercept_relaxed_route_ok = intercept_diag.get("relaxed_route_ok", 0.0)
                global_intercept_relaxed_speed_ok = intercept_diag.get("relaxed_speed_ok", 0.0)
                global_intercept_candidate_count = intercept_diag.get("candidate_count", 0.0)
                global_intercept_route_reject_count = intercept_diag.get("route_reject_count", 0.0)
                global_intercept_speed_reject_count = intercept_diag.get("speed_reject_count", 0.0)
            if intercept_candidate is not None:
                prediction_along_m = intercept_candidate["prediction_along_m"]
                prediction_lateral_m = intercept_candidate["prediction_lateral_m"]
                prediction_score = intercept_candidate["score"]
                global_intercept_tau = intercept_candidate["tau"]
                global_intercept_route_distance_m = intercept_candidate["route_distance"]
                global_intercept_route_forward_cos = intercept_candidate["route_forward_cos"]
                global_intercept_speed_margin_mps = intercept_candidate["speed_margin"]
                prediction_distance_ok = (
                    self.rear_entry_prediction_distance_min_m <= rel_distance_xy <=
                    self.rear_entry_prediction_distance_max_m
                )
                prediction_along_min_m = (
                    self.rear_entry_prediction_along_min_m
                    if self.rear_entry_require_carrier_behind
                    else -self.rear_entry_prediction_along_max_m
                )
                prediction_along_max_m = (
                    self.rear_entry_prediction_along_max_m
                    if self.rear_entry_require_carrier_behind
                    else -self.rear_entry_prediction_along_min_m
                )
                intercept_front_geometry_ok = (
                    math.isfinite(prediction_along_m) and
                    prediction_along_min_m <= prediction_along_m <= prediction_along_max_m and
                    math.isfinite(global_intercept_route_forward_cos) and
                    global_intercept_route_forward_cos >=
                    self.rear_entry_global_intercept_route_min_forward_cos and
                    math.isfinite(global_intercept_speed_margin_mps) and
                    global_intercept_speed_margin_mps >=
                    -0.25 * max(self.rear_entry_prediction_carrier_speed_mps, 1.0)
                )
                global_intercept_primary_gate_ok = (
                    intercept_front_geometry_ok and
                    math.isfinite(prediction_score) and
                    prediction_score <= self.rear_entry_global_intercept_primary_score_threshold and
                    math.isfinite(global_intercept_route_distance_m) and
                    8.0 <= global_intercept_route_distance_m <=
                    max(
                        self.rear_entry_distance_max_m,
                        self.rear_entry_prediction_secondary_distance_max_m,
                    )
                )
                prediction_score_ok = (
                    math.isfinite(prediction_score) and
                    prediction_score <= self.rear_entry_prediction_score_threshold
                )
                if prediction_score_ok:
                    self.prediction_score_history.append(prediction_score)
                else:
                    self.prediction_score_history.clear()
                if self.rear_entry_prediction_local_min_samples <= 1:
                    prediction_local_min_ok = prediction_score_ok
                elif (
                    prediction_score_ok and
                    len(self.prediction_score_history) >= self.rear_entry_prediction_local_min_samples
                ):
                    previous_scores = list(self.prediction_score_history)[:-1]
                    prediction_local_min_ok = (
                        prediction_score <= min(previous_scores)
                        if previous_scores else True
                    )
                prediction_gate_ok = (
                    abs(prediction_lateral_m) <= self.rear_entry_prediction_lateral_max_m and
                    prediction_along_min_m <= prediction_along_m <= prediction_along_max_m and
                    prediction_distance_ok and
                    prediction_score_ok and
                    prediction_local_min_ok and
                    global_intercept_route_forward_cos >= self.rear_entry_global_intercept_route_min_forward_cos and
                    global_intercept_speed_margin_mps >= -0.15 * max(self.rear_entry_prediction_carrier_speed_mps, 1.0)
                )
                prediction_secondary_score_ok = (
                    math.isfinite(prediction_score) and
                    prediction_score <= self.rear_entry_prediction_secondary_score_threshold
                )
                prediction_secondary_along_min_m = (
                    prediction_along_min_m - self.rear_entry_prediction_secondary_along_margin_m
                )
                prediction_secondary_along_max_m = (
                    prediction_along_max_m + self.rear_entry_prediction_secondary_along_margin_m
                )
                prediction_secondary_core_ok = (
                    abs(prediction_lateral_m) <= self.rear_entry_prediction_secondary_lateral_max_m and
                    prediction_secondary_along_min_m <= prediction_along_m <= prediction_secondary_along_max_m and
                    self.rear_entry_prediction_secondary_distance_min_m <= rel_distance_xy <=
                    self.rear_entry_prediction_secondary_distance_max_m and
                    prediction_secondary_score_ok
                )
                prediction_secondary_direction_ok = (
                    global_intercept_route_forward_cos >=
                    (self.rear_entry_global_intercept_route_min_forward_cos - 0.08) and
                    global_intercept_speed_margin_mps >=
                    -0.25 * max(self.rear_entry_prediction_carrier_speed_mps, 1.0) and
                    (
                        (not self.rear_entry_prediction_secondary_require_phase_gate) or
                        rear_entry_phase_ok
                    ) and
                    rear_entry_mini_vx_ok and
                    (
                        (not self.rear_entry_require_non_diverging) or
                        (
                            math.isfinite(tca_sec) and
                            (self.rear_entry_tca_min_sec - 2.0) <= tca_sec <=
                            (self.rear_entry_tca_max_sec + 2.0)
                        )
                    )
                )
                prediction_secondary_timing_ok = (
                    self.rear_entry_prediction_secondary_min_after_orbit_sec <= 0.0 or
                    (
                        math.isfinite(rear_entry_elapsed_after_orbit_sec) and
                        rear_entry_elapsed_after_orbit_sec >=
                        self.rear_entry_prediction_secondary_min_after_orbit_sec
                    )
                )
                if (
                    self.rear_entry_prediction_secondary_gate_enabled and
                    prediction_secondary_core_ok and
                    prediction_secondary_direction_ok and
                    prediction_secondary_timing_ok
                ):
                    self.prediction_secondary_hold_streak += 1
                else:
                    self.prediction_secondary_hold_streak = 0
                prediction_secondary_gate_ok = (
                    self.rear_entry_prediction_secondary_gate_enabled and
                    self.prediction_secondary_hold_streak >=
                    self.rear_entry_prediction_secondary_hold_count
                )
            else:
                self.prediction_score_history.clear()
                self.prediction_secondary_hold_streak = 0
        if intercept_candidate is None and mini_speed_xy > 1e-6:
            mini_dir_x = self.mini_odom.twist.twist.linear.x / mini_speed_xy
            mini_dir_y = self.mini_odom.twist.twist.linear.y / mini_speed_xy
            mini_lat_x = -mini_dir_y
            mini_lat_y = mini_dir_x
            horizon = self.rear_entry_prediction_horizon_sec
            mini_pred_x = self.mini_odom.pose.pose.position.x + self.mini_odom.twist.twist.linear.x * horizon
            mini_pred_y = self.mini_odom.pose.pose.position.y + self.mini_odom.twist.twist.linear.y * horizon
            to_pred_x = mini_pred_x - self.carrier_odom.pose.pose.position.x
            to_pred_y = mini_pred_y - self.carrier_odom.pose.pose.position.y
            to_pred_norm = math.hypot(to_pred_x, to_pred_y)
            if to_pred_norm > 1e-6:
                to_pred_x /= to_pred_norm
                to_pred_y /= to_pred_norm
            else:
                to_pred_x = mini_dir_x
                to_pred_y = mini_dir_y
            tangent_weight = self.rear_entry_prediction_tangent_weight
            blend_x = tangent_weight * mini_dir_x + (1.0 - tangent_weight) * to_pred_x
            blend_y = tangent_weight * mini_dir_y + (1.0 - tangent_weight) * to_pred_y
            blend_norm = math.hypot(blend_x, blend_y)
            if blend_norm > 1e-6:
                blend_x /= blend_norm
                blend_y /= blend_norm
            else:
                blend_x = mini_dir_x
                blend_y = mini_dir_y
            carrier_pred_x = self.carrier_odom.pose.pose.position.x + blend_x * self.rear_entry_prediction_carrier_speed_mps * horizon
            carrier_pred_y = self.carrier_odom.pose.pose.position.y + blend_y * self.rear_entry_prediction_carrier_speed_mps * horizon
            prediction_dx = mini_pred_x - carrier_pred_x
            prediction_dy = mini_pred_y - carrier_pred_y
            prediction_along_m = prediction_dx * mini_dir_x + prediction_dy * mini_dir_y
            prediction_lateral_m = prediction_dx * mini_lat_x + prediction_dy * mini_lat_y
            prediction_along_target_m = (
                self.rear_entry_prediction_score_along_target_m
                if self.rear_entry_require_carrier_behind
                else -self.rear_entry_prediction_score_along_target_m
            )
            prediction_along_min_m = (
                self.rear_entry_prediction_along_min_m
                if self.rear_entry_require_carrier_behind
                else -self.rear_entry_prediction_along_max_m
            )
            prediction_along_max_m = (
                self.rear_entry_prediction_along_max_m
                if self.rear_entry_require_carrier_behind
                else -self.rear_entry_prediction_along_min_m
            )
            along_scale = max(
                abs(prediction_along_max_m - prediction_along_target_m),
                abs(prediction_along_target_m - prediction_along_min_m),
                1e-6,
            )
            lateral_scale = max(self.rear_entry_prediction_lateral_max_m, 1e-6)
            prediction_score = (
                self.rear_entry_prediction_score_lateral_weight *
                (abs(prediction_lateral_m) / lateral_scale) +
                self.rear_entry_prediction_score_along_weight *
                (abs(prediction_along_m - prediction_along_target_m) / along_scale)
            )
            prediction_score += self._prediction_min_rel_z_penalty(rel_z)
            prediction_score_ok = (
                math.isfinite(prediction_score) and
                prediction_score <= self.rear_entry_prediction_score_threshold
            )
            if prediction_score_ok:
                self.prediction_score_history.append(prediction_score)
            else:
                self.prediction_score_history.clear()
            if self.rear_entry_prediction_local_min_samples <= 1:
                prediction_local_min_ok = prediction_score_ok
            elif (
                prediction_score_ok and
                len(self.prediction_score_history) >= self.rear_entry_prediction_local_min_samples
            ):
                previous_scores = list(self.prediction_score_history)[:-1]
                if previous_scores:
                    prediction_local_min_ok = prediction_score <= min(previous_scores)
                else:
                    prediction_local_min_ok = True
            prediction_gate_ok = (
                abs(prediction_lateral_m) <= self.rear_entry_prediction_lateral_max_m and
                prediction_along_min_m <= prediction_along_m <= prediction_along_max_m and
                prediction_distance_ok and
                prediction_score_ok and
                prediction_local_min_ok
            )
            prediction_secondary_score_ok = (
                math.isfinite(prediction_score) and
                prediction_score <= self.rear_entry_prediction_secondary_score_threshold
            )
            prediction_secondary_along_min_m = (
                prediction_along_min_m - self.rear_entry_prediction_secondary_along_margin_m
            )
            prediction_secondary_along_max_m = (
                prediction_along_max_m + self.rear_entry_prediction_secondary_along_margin_m
            )
            prediction_secondary_core_ok = (
                abs(prediction_lateral_m) <= self.rear_entry_prediction_secondary_lateral_max_m and
                prediction_secondary_along_min_m <= prediction_along_m <= prediction_secondary_along_max_m and
                self.rear_entry_prediction_secondary_distance_min_m <= rel_distance_xy <=
                self.rear_entry_prediction_secondary_distance_max_m and
                prediction_secondary_score_ok
            )
            prediction_secondary_direction_ok = (
                math.isfinite(mini_toward_carrier_speed) and
                math.isfinite(mini_toward_carrier_ratio) and
                mini_toward_carrier_speed <= (self.rear_entry_max_toward_speed_mps + 0.8) and
                mini_toward_carrier_ratio <= (self.rear_entry_max_toward_ratio + 0.08) and
                (
                    (not self.rear_entry_prediction_secondary_require_phase_gate) or
                    rear_entry_phase_ok
                ) and
                rear_entry_mini_vx_ok and
                (
                    (not self.rear_entry_require_non_diverging) or
                    (
                        math.isfinite(tca_sec) and
                        (self.rear_entry_tca_min_sec - 2.0) <= tca_sec <=
                        (self.rear_entry_tca_max_sec + 2.0)
                    )
                )
            )
            prediction_secondary_timing_ok = (
                self.rear_entry_prediction_secondary_min_after_orbit_sec <= 0.0 or
                (
                    math.isfinite(rear_entry_elapsed_after_orbit_sec) and
                    rear_entry_elapsed_after_orbit_sec >=
                    self.rear_entry_prediction_secondary_min_after_orbit_sec
                )
            )
            if (
                self.rear_entry_prediction_secondary_gate_enabled and
                prediction_secondary_core_ok and
                prediction_secondary_direction_ok and
                prediction_secondary_timing_ok
            ):
                self.prediction_secondary_hold_streak += 1
            else:
                self.prediction_secondary_hold_streak = 0
            prediction_secondary_gate_ok = (
                self.rear_entry_prediction_secondary_gate_enabled and
                self.prediction_secondary_hold_streak >=
                self.rear_entry_prediction_secondary_hold_count
            )
        elif intercept_candidate is None:
            self.prediction_score_history.clear()
            self.prediction_secondary_hold_streak = 0
        rear_entry_ahead_effective_ok = rear_entry_ahead_ok or intercept_front_geometry_ok
        intercept_release_timing_ok = (
            intercept_front_geometry_ok and
            (
                self.orbit_gate_completed or
                (
                    self.rear_entry_allow_orbit_progress_release and
                    rear_entry_orbit_progress_ready
                )
            )
        )
        if intercept_candidate is not None and (not global_intercept_primary_gate_ok):
            global_intercept_block_local_release = (
                math.isfinite(prediction_lateral_m) and
                abs(prediction_lateral_m) >
                max(72.0, 3.0 * self.rear_entry_prediction_secondary_lateral_max_m) and
                math.isfinite(prediction_score) and
                prediction_score >
                (self.rear_entry_prediction_secondary_score_threshold + 1.8) and
                math.isfinite(global_intercept_route_distance_m) and
                global_intercept_route_distance_m > 80.0 and
                (
                    (not math.isfinite(rear_entry_elapsed_after_orbit_sec)) or
                    rear_entry_elapsed_after_orbit_sec < 18.0
                )
            )
        if (
            self.rear_entry_energy_early_release_enabled and
            (not rear_entry_energy_timing_ok) and
            math.isfinite(rear_entry_elapsed_after_orbit_sec) and
            rear_entry_elapsed_after_orbit_sec >=
            self.rear_entry_energy_early_release_min_after_orbit_sec
        ):
            prediction_early_score_ok = (
                math.isfinite(prediction_score) and
                prediction_score <= self.rear_entry_energy_early_release_score_threshold
            )
            if math.isfinite(prediction_along_m) and math.isfinite(prediction_along_min_m) and math.isfinite(prediction_along_max_m):
                prediction_early_along_ok = (
                    prediction_along_m >=
                    prediction_along_min_m - self.rear_entry_energy_early_release_along_margin_m and
                    prediction_along_m <=
                    prediction_along_max_m + self.rear_entry_energy_early_release_along_margin_m
                )
            prediction_early_geom_ok = (
                math.isfinite(prediction_lateral_m) and
                abs(prediction_lateral_m) <= self.rear_entry_energy_early_release_lateral_max_m and
                prediction_early_along_ok and
                prediction_distance_ok
            )
            energy_early_release_candidate = (
                prediction_early_score_ok and
                prediction_early_geom_ok and
                rear_entry_orbit_progress_ready and
                intercept_front_geometry_ok
            )
        if energy_early_release_candidate:
            self.energy_early_release_hold_streak += 1
        else:
            self.energy_early_release_hold_streak = 0
        energy_early_release_ok = (
            energy_early_release_candidate and
            self.energy_early_release_hold_streak >=
            self.rear_entry_energy_early_release_hold_count
        )
        rear_entry_energy_timing_ok_effective = (
            rear_entry_energy_timing_ok or
            energy_early_release_ok or
            intercept_release_timing_ok
        )
        smoke_early_reject_active = (
            self.rear_entry_smoke_early_reject_tca_max_sec > 0.0 and
            self.rear_entry_smoke_early_reject_speed_min_mps > 0.0 and
            self.rear_entry_smoke_early_reject_pred_lat_abs_min_m > 0.0 and
            math.isfinite(tca_sec) and
            math.isfinite(rel_speed_xy) and
            math.isfinite(prediction_lateral_m) and
            tca_sec <= self.rear_entry_smoke_early_reject_tca_max_sec and
            rel_speed_xy >= self.rear_entry_smoke_early_reject_speed_min_mps and
            abs(prediction_lateral_m) >= self.rear_entry_smoke_early_reject_pred_lat_abs_min_m
        )
        smoke_prediction_reject_active = (
            self.rear_entry_smoke_reject_prediction_score_min > 0.0 and
            self.rear_entry_smoke_reject_diag_margin_max_mps > 0.0 and
            math.isfinite(prediction_score) and
            math.isfinite(global_intercept_relaxed_speed_margin_mps) and
            prediction_score >= self.rear_entry_smoke_reject_prediction_score_min and
            global_intercept_relaxed_speed_margin_mps <=
            self.rear_entry_smoke_reject_diag_margin_max_mps
        )
        prediction_gate_effective_ok = (
            (
                (prediction_gate_ok or prediction_secondary_gate_ok) and
                (not global_intercept_block_local_release)
            ) or
            global_intercept_primary_gate_ok
        )
        if self.rear_entry_require_global_primary_gate:
            prediction_gate_effective_ok = global_intercept_primary_gate_ok
        rear_entry_distance_ok = (
            (
                rel_distance_xy >= self.rear_entry_distance_min_m and
                rel_distance_xy <= self.rear_entry_distance_max_m
            ) or
            global_intercept_primary_gate_ok
        )
        rear_entry_direction_ok = (
            self.rear_entry_prediction_primary_gate or
            (
                math.isfinite(mini_toward_carrier_speed) and
                math.isfinite(mini_toward_carrier_ratio) and
                mini_toward_carrier_speed <= self.rear_entry_max_toward_speed_mps and
                mini_toward_carrier_ratio <= self.rear_entry_max_toward_ratio and
                (
                    (not self.rear_entry_require_non_diverging) or
                    (
                        math.isfinite(tca_sec) and
                        self.rear_entry_tca_min_sec <= tca_sec <= self.rear_entry_tca_max_sec
                    )
                ) and
                rear_entry_phase_ok and
                rear_entry_mini_vx_ok
            ) or
            intercept_front_geometry_ok
        )
        release_on_orbit_completion_ok = (
            self.release_on_orbit_completion and
            self.orbit_gate_completed and
            self.orbit_gate_completed_elapsed_sec is not None and
            (elapsed - self.orbit_gate_completed_elapsed_sec) >= self.release_after_orbit_delay_sec
        )
        rear_entry_window_ok = (
            rel_z >= self.relative_z_min_m and
            rel_speed_xy >= self.relative_speed_min_mps and
            rear_entry_distance_ok and
            geometry_cluster_ok and
            rear_entry_ahead_effective_ok and
            rear_entry_energy_timing_ok_effective and
            (not smoke_early_reject_active) and
            (not smoke_prediction_reject_active) and
            (
                (not self.rear_entry_use_prediction_gate) or
                prediction_gate_effective_ok
            ) and
            rear_entry_orbit_progress_ready and
            rear_entry_direction_ok
        )
        geometry_window_ok = rear_entry_window_ok if self.prefer_rear_entry else legacy_window_ok
        window_ok = release_on_orbit_completion_ok or geometry_window_ok

        if window_ok:
            self.hold_count += 1
        else:
            self.hold_count = 0

        if int(elapsed * 10) % 10 == 0:
            self.get_logger().info(
                "window_check "
                f"dist_xy={rel_distance_xy:.2f} rel_z={rel_z:.2f} "
                f"carrier_z={carrier_z:.2f} carrier_vz={carrier_vz:.2f} "
                f"prehold_ok={int(carrier_prehold_ready)} "
                f"speed_xy={rel_speed_xy:.2f} align={alignment:.3f} tca={tca_sec:.2f} "
                f"toward={mini_toward_carrier_speed:.2f} toward_ratio={mini_toward_carrier_ratio:.2f} "
                f"ahead={carrier_ahead_along_mini:.2f} "
                f"ahead_ok={int(rear_entry_ahead_ok)} "
                f"ahead_eff_ok={int(rear_entry_ahead_effective_ok)} "
                f"energy_after_orbit={rear_entry_elapsed_after_orbit_sec:.2f} "
                f"energy_ok={int(rear_entry_energy_timing_ok)} "
                f"energy_relaxed={int(rear_entry_energy_relaxed)} "
                f"int_release_ok={int(intercept_release_timing_ok)} "
                f"energy_early_cand={int(energy_early_release_candidate)} "
                f"energy_early_hold={self.energy_early_release_hold_streak}/"
                f"{self.rear_entry_energy_early_release_hold_count} "
                f"energy_early_ok={int(energy_early_release_ok)} "
                f"phase_err_deg={math.degrees(mini_orbit_phase_error):.1f} "
                f"mini_vx={self.mini_odom.twist.twist.linear.x:.2f} "
                f"pred_along={prediction_along_m:.2f} pred_lat={prediction_lateral_m:.2f} "
                f"pred_score={prediction_score:.2f} pred_dist_ok={int(prediction_distance_ok)} "
                f"int_tau={global_intercept_tau:.2f} int_route={global_intercept_route_distance_m:.2f} "
                f"int_cos={global_intercept_route_forward_cos:.2f} int_margin={global_intercept_speed_margin_mps:.2f} "
                f"int_front_ok={int(intercept_front_geometry_ok)} "
                f"int_gate_ok={int(global_intercept_primary_gate_ok)} "
                f"diag_tau={global_intercept_relaxed_tau:.2f} "
                f"diag_route={global_intercept_relaxed_route_distance_m:.2f} "
                f"diag_cos={global_intercept_relaxed_route_forward_cos:.2f} "
                f"diag_margin={global_intercept_relaxed_speed_margin_mps:.2f} "
                f"diag_lat={global_intercept_relaxed_prediction_lateral_m:.2f} "
                f"diag_score={global_intercept_relaxed_score:.2f} "
                f"diag_path={global_intercept_relaxed_path_ratio:.2f} "
                f"diag_route_ok={int(global_intercept_relaxed_route_ok)} "
                f"diag_speed_ok={int(global_intercept_relaxed_speed_ok)} "
                f"diag_cand={int(global_intercept_candidate_count)} "
                f"diag_route_rej={int(global_intercept_route_reject_count)} "
                f"diag_speed_rej={int(global_intercept_speed_reject_count)} "
                f"int_local_veto={int(global_intercept_block_local_release)} "
                f"pred_local_min_ok={int(prediction_local_min_ok)} "
                f"pred_sec_core_ok={int(prediction_secondary_core_ok)} "
                f"pred_sec_dir_ok={int(prediction_secondary_direction_ok)} "
                f"pred_sec_hold={self.prediction_secondary_hold_streak}/"
                f"{self.rear_entry_prediction_secondary_hold_count} "
                f"pred_sec_ok={int(prediction_secondary_gate_ok)} "
                f"smoke_early_reject={int(smoke_early_reject_active)} "
                f"smoke_pred_reject={int(smoke_prediction_reject_active)} "
                f"orbit_release_ok={int(release_on_orbit_completion_ok)} "
                f"cluster={geometry_cluster_score:.3f} "
                f"hold={self.hold_count}/{self.hold_count_required}"
            )

        orbit_gate_released = self.orbit_gate_completed
        if (
            self.prefer_rear_entry and
            self.rear_entry_allow_orbit_progress_release and
            rear_entry_orbit_progress_ready
        ):
            orbit_gate_released = True

        if self.require_orbit_completion_before_start and not orbit_gate_released:
            self.hold_count = 0
            if timeout_elapsed > self.timeout_sec:
                if self.fallback_immediate:
                    self.get_logger().warn(
                        "Timeout before orbit gate completion, publishing START anyway"
                    )
                    self._publish_start("timeout_before_orbit_gate")
                else:
                    self.get_logger().error("Timeout before orbit gate completion")
                    rclpy.shutdown()
                    sys.exit(1)
            return

        if self.hold_count >= self.hold_count_required:
            self.get_logger().info(
                "Window accepted "
                f"dist_xy={rel_distance_xy:.2f} rel_z={rel_z:.2f} "
                f"carrier_z={carrier_z:.2f} carrier_vz={carrier_vz:.2f} "
                f"prehold_ok={int(carrier_prehold_ready)} "
                f"speed_xy={rel_speed_xy:.2f} align={alignment:.3f} tca={tca_sec:.2f} "
                f"ahead={carrier_ahead_along_mini:.2f} "
                f"energy_after_orbit={rear_entry_elapsed_after_orbit_sec:.2f} "
                f"phase_err_deg={math.degrees(mini_orbit_phase_error):.1f} "
                f"pred_along={prediction_along_m:.2f} pred_lat={prediction_lateral_m:.2f} "
                f"pred_score={prediction_score:.2f} "
                f"pred_dist_ok={int(prediction_distance_ok)} "
                f"int_tau={global_intercept_tau:.2f} int_route={global_intercept_route_distance_m:.2f} "
                f"int_cos={global_intercept_route_forward_cos:.2f} int_margin={global_intercept_speed_margin_mps:.2f} "
                f"int_front_ok={int(intercept_front_geometry_ok)} "
                f"diag_tau={global_intercept_relaxed_tau:.2f} "
                f"diag_route={global_intercept_relaxed_route_distance_m:.2f} "
                f"diag_cos={global_intercept_relaxed_route_forward_cos:.2f} "
                f"diag_margin={global_intercept_relaxed_speed_margin_mps:.2f} "
                f"diag_lat={global_intercept_relaxed_prediction_lateral_m:.2f} "
                f"diag_score={global_intercept_relaxed_score:.2f} "
                f"diag_path={global_intercept_relaxed_path_ratio:.2f} "
                f"diag_route_ok={int(global_intercept_relaxed_route_ok)} "
                f"diag_speed_ok={int(global_intercept_relaxed_speed_ok)} "
                f"diag_cand={int(global_intercept_candidate_count)} "
                f"diag_route_rej={int(global_intercept_route_reject_count)} "
                f"diag_speed_rej={int(global_intercept_speed_reject_count)} "
                f"pred_local_min_ok={int(prediction_local_min_ok)} "
                f"pred_sec_ok={int(prediction_secondary_gate_ok)} "
                f"smoke_early_reject={int(smoke_early_reject_active)} "
                f"smoke_pred_reject={int(smoke_prediction_reject_active)}"
            )
            self._publish_start("window")
            return

        if timeout_elapsed > self.timeout_sec:
            if self.fallback_immediate:
                self.get_logger().warn(
                    "Timeout waiting for window, publishing START anyway"
                )
                self._publish_start("timeout")
            else:
                self.get_logger().error("Timeout waiting for docking window")
                rclpy.shutdown()
                sys.exit(1)

    def _publish_start(self, reason: str) -> None:
        msg = DockingCommand()
        msg.command = "START"
        for _ in range(max(self.publish_repeats, 1)):
            self.command_pub.publish(msg)
            self.command_latched_pub.publish(msg)
        self.command_sent = True
        self.get_logger().info(f"START published reason={reason}")
        time.sleep(0.2)
        self._shutdown_once()

    def _shutdown_once(self) -> None:
        if rclpy.ok():
            rclpy.shutdown()
        raise SystemExit(0)

    def _update_orbit_gate(self) -> None:
        if self.orbit_gate_completed or self.mini_odom is None:
            return

        x = float(self.mini_odom.pose.pose.position.x)
        y = float(self.mini_odom.pose.pose.position.y)
        z = float(self.mini_odom.pose.pose.position.z)
        dx = x - self.orbit_center_x
        dy = y - self.orbit_center_y
        radius = math.hypot(dx, dy)
        angle = math.atan2(dy, dx)
        ready = (
            z >= self.orbit_gate_ready_altitude and
            abs(radius - self.orbit_radius) <= self.orbit_gate_ready_radius_tolerance
        )

        if not self.orbit_gate_tracking_active:
            if ready:
                self.orbit_gate_ready_samples += 1
                if self.orbit_gate_ready_samples >= self.orbit_gate_min_valid_samples:
                    self.orbit_gate_tracking_active = True
                    self.orbit_gate_initial_angle = angle
                    self.orbit_gate_previous_angle = angle
                    self.orbit_gate_accumulated_angle = 0.0
                    self.orbit_gate_last_reported_quarter = 0
                    self.get_logger().info(
                        "Orbit gate armed "
                        f"altitude={z:.2f} radius={radius:.2f} "
                        f"laps_required={self.orbit_gate_required_laps:.2f}"
                    )
            else:
                self.orbit_gate_ready_samples = 0
            return

        if self.orbit_gate_previous_angle is None or self.orbit_gate_initial_angle is None:
            self.orbit_gate_initial_angle = angle
            self.orbit_gate_previous_angle = angle
            return

        delta = self._normalize_angle(angle - self.orbit_gate_previous_angle)
        self.orbit_gate_previous_angle = angle
        self.orbit_gate_accumulated_angle += delta
        completed_laps = abs(self.orbit_gate_accumulated_angle) / (2.0 * math.pi)
        reported_quarter = int(completed_laps * 4.0)
        if reported_quarter > self.orbit_gate_last_reported_quarter:
            self.orbit_gate_last_reported_quarter = reported_quarter
            self.get_logger().info(
                "Orbit gate progress "
                f"laps={completed_laps:.2f} altitude={z:.2f} radius={radius:.2f}"
            )

        returned_to_start = (
            abs(self._normalize_angle(angle - self.orbit_gate_initial_angle)) <=
            self.orbit_gate_return_tolerance_rad
        )
        if (
            completed_laps >= self.orbit_gate_required_laps and
            abs(self.orbit_gate_accumulated_angle) >= self.orbit_gate_min_accumulated_angle_rad and
            returned_to_start
        ):
            self.orbit_gate_completed = True
            self.hold_count = 0
            elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
            self.orbit_gate_completed_elapsed_sec = elapsed
            self.get_logger().info(
                "Orbit gate completed "
                f"laps={completed_laps:.2f} altitude={z:.2f} radius={radius:.2f}"
            )

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def _prediction_min_rel_z_penalty(self, rel_z: float) -> float:
        if (
            self.rear_entry_prediction_score_min_rel_z_weight <= 0.0 or
            self.rear_entry_prediction_score_min_rel_z_target_m <= 0.0 or
            (not math.isfinite(rel_z))
        ):
            return 0.0
        rel_z_deficit = max(0.0, self.rear_entry_prediction_score_min_rel_z_target_m - rel_z)
        return (
            self.rear_entry_prediction_score_min_rel_z_weight *
            (rel_z_deficit / self.rear_entry_prediction_score_min_rel_z_scale_m)
        )

    @staticmethod
    def _geometry_cluster_score(
        sample: tuple[float, float, float, float],
        center: list[float],
        spread: list[float],
    ) -> float:
        score = 0.0
        for value, target, scale in zip(sample, center, spread):
            score += ((value - target) / max(scale, 1e-6)) ** 2
        return score

    def _mini_energy_healthy(self) -> bool:
        if not self.require_mini_energy_healthy:
            return True
        healthy = (
            math.isfinite(self.mini_true_airspeed_mps) and
            math.isfinite(self.mini_underspeed_ratio) and
            self.mini_true_airspeed_mps >= self.health_min_true_airspeed_mps and
            self.mini_underspeed_ratio <= self.health_max_underspeed_ratio
        )
        if healthy:
            self.mini_health_good_samples += 1
        else:
            self.mini_health_good_samples = 0

        ready = self.mini_health_good_samples >= self.health_min_samples
        if self.mini_health_announced != ready:
            self.mini_health_announced = ready
            if ready:
                self.get_logger().info(
                    "Window starter mini-energy healthy "
                    f"tas={self.mini_true_airspeed_mps:.2f} "
                    f"underspeed={self.mini_underspeed_ratio:.3f}"
                )
            else:
                self.get_logger().info(
                    "Window starter waiting for mini-energy health "
                    f"tas={self.mini_true_airspeed_mps:.2f} "
                    f"underspeed={self.mini_underspeed_ratio:.3f}"
                )
        return ready

    def _carrier_prehold_ready(self) -> bool:
        if not self.carrier_prehold_required:
            return True
        if self.carrier_odom is None:
            return False

        carrier_z = float(self.carrier_odom.pose.pose.position.z)
        carrier_vz = float(self.carrier_odom.twist.twist.linear.z)
        ready_sample = (
            math.isfinite(carrier_z) and
            math.isfinite(carrier_vz) and
            carrier_z >= self.carrier_prehold_min_altitude_m and
            abs(carrier_vz) <= self.carrier_prehold_max_abs_vz_mps
        )
        if ready_sample:
            self.carrier_prehold_good_samples += 1
        else:
            self.carrier_prehold_good_samples = 0

        ready = self.carrier_prehold_good_samples >= self.carrier_prehold_min_samples
        if self.carrier_prehold_announced != ready:
            self.carrier_prehold_announced = ready
            if ready:
                self.get_logger().info(
                    "Window starter carrier prehold ready "
                    f"z={carrier_z:.2f} vz={carrier_vz:.2f} "
                    f"min_alt={self.carrier_prehold_min_altitude_m:.2f}"
                )
            else:
                self.get_logger().info(
                    "Window starter waiting for carrier prehold "
                    f"z={carrier_z:.2f} vz={carrier_vz:.2f} "
                    f"min_alt={self.carrier_prehold_min_altitude_m:.2f}"
                )
        return ready


def main() -> None:
    rclpy.init()
    node = DockingWindowStarter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
