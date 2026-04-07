#!/usr/bin/env python3

import csv
import os
import time
import math
from pathlib import Path

import rclpy
from easydocking_msgs.msg import DockingStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Float64MultiArray

try:
    from px4_msgs.msg import (
        AirspeedValidated,
        FixedWingLongitudinalSetpoint,
        TecsStatus,
        VehicleAttitudeSetpoint,
        VehicleGlobalPosition,
        VehicleLocalPosition,
    )
except ImportError:  # pragma: no cover
    AirspeedValidated = None
    FixedWingLongitudinalSetpoint = None
    TecsStatus = None
    VehicleAttitudeSetpoint = None
    VehicleGlobalPosition = None
    VehicleLocalPosition = None


def quaternion_to_euler_deg(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)

    return tuple(math.degrees(angle) for angle in (roll, pitch, yaw))


class ExperimentLogger(Node):
    def __init__(self) -> None:
        super().__init__("experiment_logger")
        self.declare_parameter("output_dir", "")
        self.declare_parameter("duration_sec", 30.0)
        self.declare_parameter("post_completed_settle_sec", 0.6)
        self.declare_parameter("post_completed_rows", 12)
        self.declare_parameter("prototype_capture_distance_m", 1.25)
        self.declare_parameter("prototype_capture_rel_speed_mps", 0.45)
        self.declare_parameter("prototype_capture_sync_score", 0.90)
        self.declare_parameter("prototype_capture_max_line_lateral_error_m", 0.45)
        self.declare_parameter("prototype_capture_max_height_error_m", 0.20)
        self.declare_parameter("prototype_capture_hold_rows", 8)
        self.declare_parameter("mini_px4_namespace", "/px4_2")

        output_dir = self.get_parameter("output_dir").value
        if not output_dir:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            output_dir = f"/home/hw/easydocking/results/{timestamp}"
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.duration_sec = float(self.get_parameter("duration_sec").value)
        self.post_completed_settle_sec = float(
            self.get_parameter("post_completed_settle_sec").value
        )
        self.post_completed_rows = int(self.get_parameter("post_completed_rows").value)
        self.prototype_capture_distance_m = float(
            self.get_parameter("prototype_capture_distance_m").value
        )
        self.prototype_capture_rel_speed_mps = float(
            self.get_parameter("prototype_capture_rel_speed_mps").value
        )
        self.prototype_capture_sync_score = float(
            self.get_parameter("prototype_capture_sync_score").value
        )
        self.prototype_capture_max_line_lateral_error_m = float(
            self.get_parameter("prototype_capture_max_line_lateral_error_m").value
        )
        self.prototype_capture_max_height_error_m = float(
            self.get_parameter("prototype_capture_max_height_error_m").value
        )
        self.prototype_capture_hold_rows = int(
            self.get_parameter("prototype_capture_hold_rows").value
        )
        self.start_wall = time.time()
        self.start_ros_time = None
        self.latest_phase = "IDLE"
        self.rows_written = 0
        self.completed_wall_time = None
        self.completed_rows_written = 0
        self.prototype_capture_rows = 0
        self.mini_px4_namespace = str(self.get_parameter("mini_px4_namespace").value).rstrip("/")

        self.csv_path = self.output_dir / "docking_log.csv"
        self.metadata_path = self.output_dir / "metadata.txt"
        self.csv_file = self.csv_path.open("w", newline="", encoding="utf-8")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow([
            "t",
            "phase",
            "relative_distance",
            "rel_x",
            "rel_y",
            "rel_z",
            "rel_vx",
            "rel_vy",
            "rel_vz",
            "carrier_x",
            "carrier_y",
            "carrier_z",
            "carrier_vx",
            "carrier_vy",
            "carrier_vz",
            "carrier_roll_deg",
            "carrier_pitch_deg",
            "carrier_yaw_deg",
            "mini_x",
            "mini_y",
            "mini_z",
            "mini_vx",
            "mini_vy",
            "mini_vz",
            "mini_roll_deg",
            "mini_pitch_deg",
            "mini_yaw_deg",
            "carrier_sp_x",
            "carrier_sp_y",
            "carrier_sp_z",
            "carrier_cmd_vx",
            "carrier_cmd_vy",
            "carrier_cmd_vz",
            "mini_target_x",
            "mini_target_y",
            "mini_target_z",
            "measured_rel_x",
            "measured_rel_y",
            "measured_rel_z",
            "measured_rel_distance",
            "measured_rel_vx",
            "measured_rel_vy",
            "measured_rel_vz",
            "measured_rel_speed",
            "mini_px4_true_airspeed_mps",
            "mini_px4_throttle_filtered",
            "mini_px4_global_alt_m",
            "mini_px4_global_alt_valid",
            "mini_px4_local_z_m",
            "mini_px4_local_vz_mps",
            "mini_px4_local_ref_alt_m",
            "mini_px4_local_z_global_valid",
            "mini_px4_local_alt_from_ref_m",
            "mini_px4_alt_source_delta_m",
            "mini_tecs_altitude_sp_m",
            "mini_tecs_altitude_reference_m",
            "mini_tecs_height_rate_reference_mps",
            "mini_tecs_height_rate_sp_mps",
            "mini_tecs_height_rate_mps",
            "mini_tecs_true_airspeed_sp_mps",
            "mini_tecs_true_airspeed_filtered_mps",
            "mini_tecs_throttle_sp",
            "mini_tecs_throttle_trim",
            "mini_tecs_pitch_sp_deg",
            "mini_tecs_underspeed_ratio",
            "mini_tecs_fast_descend_ratio",
            "mini_tecs_total_energy_rate_sp",
            "mini_tecs_total_energy_rate",
            "mini_fw_altitude_sp_m",
            "mini_fw_height_rate_sp_mps",
            "mini_fw_equivalent_airspeed_sp_mps",
            "mini_fw_pitch_direct_deg",
            "mini_fw_throttle_direct",
            "mini_attitude_sp_thrust_x",
            "mini_terminal_straight_active",
            "mini_terminal_sync_active",
            "mini_terminal_sync_score",
            "mini_terminal_sync_best_score",
            "mini_terminal_sync_release_ready",
            "mini_terminal_sync_good_count",
            "mini_terminal_sync_plateau_count",
            "mini_terminal_sync_cluster_code",
            "mini_terminal_sync_distance_xy",
            "mini_terminal_sync_line_progress",
            "mini_terminal_sync_line_lateral_error",
            "mini_terminal_sync_height_error",
            "mini_terminal_sync_distance_score",
            "mini_terminal_sync_progress_score",
            "mini_terminal_sync_lateral_score",
            "mini_terminal_sync_height_score",
            "mini_handoff_tangent_active",
            "mini_handoff_distance_xy",
            "mini_handoff_line_progress",
            "mini_handoff_line_lateral_error",
            "mini_handoff_height_error",
            "mini_handoff_geom_ready",
            "mini_energy_guard_active",
            "mini_energy_guard_bad_airspeed",
            "mini_energy_guard_bad_underspeed",
            "mini_energy_guard_true_airspeed_mps",
            "mini_energy_guard_underspeed_ratio",
            "mini_energy_guard_recover_time_remaining_sec",
            "controller_phase_code",
            "controller_tracking_horizontal_distance",
            "controller_tracking_along_error",
            "controller_tracking_lateral_error",
            "controller_tracking_along_speed",
            "controller_tracking_lateral_speed",
            "controller_terminal_distance",
            "controller_terminal_along_error",
            "controller_terminal_lateral_error",
            "controller_terminal_vertical_error",
            "controller_target_along_speed",
            "controller_target_lateral_speed",
            "controller_relative_speed",
            "controller_min_docking_distance",
            "controller_rendezvous_corridor_active",
            "controller_passive_target_mode",
            "controller_corridor_release_score",
            "controller_corridor_release_armed",
            "controller_corridor_release_accept_counter",
            "controller_completion_hold_counter",
        ])

        self.carrier_position = [0.0, 0.0, 0.0]
        self.mini_position = [0.0, 0.0, 0.0]
        self.carrier_velocity = [0.0, 0.0, 0.0]
        self.mini_velocity = [0.0, 0.0, 0.0]
        self.carrier_euler_deg = [0.0, 0.0, 0.0]
        self.mini_euler_deg = [0.0, 0.0, 0.0]
        self.carrier_odom_received = False
        self.mini_odom_received = False
        self.carrier_setpoint = [0.0, 0.0, 0.0]
        self.carrier_cmd_velocity = [math.nan, math.nan, math.nan]
        self.mini_target = [0.0, 0.0, 0.0]
        self.latest_status = None
        self.mini_px4_true_airspeed = math.nan
        self.mini_px4_throttle_filtered = math.nan
        self.mini_px4_global_alt = math.nan
        self.mini_px4_global_alt_valid = 0.0
        self.mini_px4_local_z = math.nan
        self.mini_px4_local_vz = math.nan
        self.mini_px4_local_ref_alt = math.nan
        self.mini_px4_local_z_global_valid = 0.0
        self.mini_tecs_altitude_sp = math.nan
        self.mini_tecs_altitude_reference = math.nan
        self.mini_tecs_height_rate_reference = math.nan
        self.mini_tecs_height_rate_sp = math.nan
        self.mini_tecs_height_rate = math.nan
        self.mini_tecs_true_airspeed_sp = math.nan
        self.mini_tecs_true_airspeed_filtered = math.nan
        self.mini_tecs_throttle_sp = math.nan
        self.mini_tecs_throttle_trim = math.nan
        self.mini_tecs_pitch_sp_deg = math.nan
        self.mini_tecs_underspeed_ratio = math.nan
        self.mini_tecs_fast_descend_ratio = math.nan
        self.mini_tecs_total_energy_rate_sp = math.nan
        self.mini_tecs_total_energy_rate = math.nan
        self.mini_fw_altitude_sp = math.nan
        self.mini_fw_height_rate_sp = math.nan
        self.mini_fw_equivalent_airspeed_sp = math.nan
        self.mini_fw_pitch_direct_deg = math.nan
        self.mini_fw_throttle_direct = math.nan
        self.mini_attitude_sp_thrust_x = math.nan
        self.mini_terminal_sync_debug = [math.nan] * 16
        self.mini_handoff_debug = [math.nan] * 6
        self.mini_energy_debug = [math.nan] * 6
        self.controller_debug = [math.nan] * 20

        self.create_subscription(DockingStatus, "/docking/status", self._status_cb, 10)
        self.create_subscription(Odometry, "/carrier/odom", self._carrier_cb, 10)
        self.create_subscription(Odometry, "/mini/odom", self._mini_cb, 10)
        self.create_subscription(PoseStamped, "/carrier/setpoint/pose", self._carrier_sp_cb, 10)
        self.create_subscription(
            TwistStamped, "/carrier/setpoint/velocity", self._carrier_cmd_vel_cb, 10
        )
        self.create_subscription(PoseStamped, "/mini/glide_target", self._mini_target_cb, 10)
        self.create_subscription(
            Float64MultiArray,
            "/mini/terminal_sync_debug",
            self._mini_terminal_sync_debug_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/mini/handoff_debug",
            self._mini_handoff_debug_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/mini/energy_debug",
            self._mini_energy_debug_cb,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            "/docking/controller_debug",
            self._controller_debug_cb,
            10,
        )

        if all(obj is not None for obj in [
            AirspeedValidated,
            FixedWingLongitudinalSetpoint,
            TecsStatus,
            VehicleAttitudeSetpoint,
            VehicleGlobalPosition,
            VehicleLocalPosition,
        ]):
            px4_qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.create_subscription(
                AirspeedValidated,
                f"{self.mini_px4_namespace}/fmu/out/airspeed_validated",
                self._mini_airspeed_cb,
                px4_qos,
            )
            self.create_subscription(
                AirspeedValidated,
                f"{self.mini_px4_namespace}/fmu/out/airspeed_validated_v1",
                self._mini_airspeed_cb,
                px4_qos,
            )
            self.create_subscription(
                VehicleGlobalPosition,
                f"{self.mini_px4_namespace}/fmu/out/vehicle_global_position",
                self._mini_global_pos_cb,
                px4_qos,
            )
            self.create_subscription(
                VehicleGlobalPosition,
                f"{self.mini_px4_namespace}/fmu/out/vehicle_global_position_v1",
                self._mini_global_pos_cb,
                px4_qos,
            )
            self.create_subscription(
                VehicleLocalPosition,
                f"{self.mini_px4_namespace}/fmu/out/vehicle_local_position",
                self._mini_local_pos_cb,
                px4_qos,
            )
            self.create_subscription(
                VehicleLocalPosition,
                f"{self.mini_px4_namespace}/fmu/out/vehicle_local_position_v1",
                self._mini_local_pos_cb,
                px4_qos,
            )
            self.create_subscription(
                TecsStatus,
                f"{self.mini_px4_namespace}/fmu/out/tecs_status",
                self._mini_tecs_cb,
                px4_qos,
            )
            self.create_subscription(
                TecsStatus,
                f"{self.mini_px4_namespace}/fmu/out/tecs_status_v1",
                self._mini_tecs_cb,
                px4_qos,
            )
            self.create_subscription(
                FixedWingLongitudinalSetpoint,
                f"{self.mini_px4_namespace}/fmu/out/fixed_wing_longitudinal_setpoint",
                self._mini_fw_longitudinal_sp_cb,
                px4_qos,
            )
            self.create_subscription(
                FixedWingLongitudinalSetpoint,
                f"{self.mini_px4_namespace}/fmu/out/fixed_wing_longitudinal_setpoint_v1",
                self._mini_fw_longitudinal_sp_cb,
                px4_qos,
            )
            self.create_subscription(
                VehicleAttitudeSetpoint,
                f"{self.mini_px4_namespace}/fmu/out/vehicle_attitude_setpoint",
                self._mini_attitude_sp_cb,
                px4_qos,
            )
            self.create_subscription(
                VehicleAttitudeSetpoint,
                f"{self.mini_px4_namespace}/fmu/out/vehicle_attitude_setpoint_v1",
                self._mini_attitude_sp_cb,
                px4_qos,
            )
        self.timer = self.create_timer(0.05, self._flush_row)

    def _carrier_cb(self, msg: Odometry) -> None:
        self.carrier_odom_received = True
        self.carrier_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.carrier_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        self.carrier_euler_deg = list(quaternion_to_euler_deg(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ))

    def _mini_cb(self, msg: Odometry) -> None:
        self.mini_odom_received = True
        self.mini_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        ]
        self.mini_velocity = [
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ]
        self.mini_euler_deg = list(quaternion_to_euler_deg(
            msg.pose.pose.orientation.w,
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
        ))

    def _carrier_sp_cb(self, msg: PoseStamped) -> None:
        self.carrier_setpoint = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

    def _carrier_cmd_vel_cb(self, msg: TwistStamped) -> None:
        self.carrier_cmd_velocity = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ]

    def _mini_target_cb(self, msg: PoseStamped) -> None:
        self.mini_target = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ]

    def _status_cb(self, msg: DockingStatus) -> None:
        self.latest_status = msg
        self.latest_phase = msg.phase
        if self.start_ros_time is None:
            self.start_ros_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def _mini_airspeed_cb(self, msg: AirspeedValidated) -> None:
        self.mini_px4_true_airspeed = float(msg.true_airspeed_m_s)
        self.mini_px4_throttle_filtered = float(msg.throttle_filtered)

    def _mini_global_pos_cb(self, msg: VehicleGlobalPosition) -> None:
        self.mini_px4_global_alt = float(msg.alt)
        self.mini_px4_global_alt_valid = 1.0 if bool(msg.alt_valid) else 0.0

    def _mini_local_pos_cb(self, msg: VehicleLocalPosition) -> None:
        self.mini_px4_local_z = float(msg.z)
        self.mini_px4_local_vz = float(msg.vz)
        self.mini_px4_local_ref_alt = float(msg.ref_alt)
        self.mini_px4_local_z_global_valid = 1.0 if bool(msg.z_global) else 0.0

    def _mini_tecs_cb(self, msg: TecsStatus) -> None:
        self.mini_tecs_altitude_sp = float(msg.altitude_sp)
        self.mini_tecs_altitude_reference = float(msg.altitude_reference)
        self.mini_tecs_height_rate_reference = float(msg.height_rate_reference)
        self.mini_tecs_height_rate_sp = float(msg.height_rate_setpoint)
        self.mini_tecs_height_rate = float(msg.height_rate)
        self.mini_tecs_true_airspeed_sp = float(msg.true_airspeed_sp)
        self.mini_tecs_true_airspeed_filtered = float(msg.true_airspeed_filtered)
        self.mini_tecs_throttle_sp = float(msg.throttle_sp)
        self.mini_tecs_throttle_trim = float(msg.throttle_trim)
        self.mini_tecs_pitch_sp_deg = math.degrees(float(msg.pitch_sp_rad))
        self.mini_tecs_underspeed_ratio = float(msg.underspeed_ratio)
        self.mini_tecs_fast_descend_ratio = float(msg.fast_descend_ratio)
        self.mini_tecs_total_energy_rate_sp = float(msg.total_energy_rate_sp)
        self.mini_tecs_total_energy_rate = float(msg.total_energy_rate)

    def _mini_fw_longitudinal_sp_cb(self, msg: FixedWingLongitudinalSetpoint) -> None:
        self.mini_fw_altitude_sp = float(msg.altitude)
        self.mini_fw_height_rate_sp = float(msg.height_rate)
        self.mini_fw_equivalent_airspeed_sp = float(msg.equivalent_airspeed)
        self.mini_fw_pitch_direct_deg = math.degrees(float(msg.pitch_direct))
        self.mini_fw_throttle_direct = float(msg.throttle_direct)

    def _mini_attitude_sp_cb(self, msg: VehicleAttitudeSetpoint) -> None:
        self.mini_attitude_sp_thrust_x = float(msg.thrust_body[0])

    def _mini_terminal_sync_debug_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 16:
            return
        self.mini_terminal_sync_debug = [float(value) for value in msg.data[:16]]

    def _mini_handoff_debug_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            return
        self.mini_handoff_debug = [float(value) for value in msg.data[:6]]

    def _mini_energy_debug_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 6:
            return
        self.mini_energy_debug = [float(value) for value in msg.data[:6]]

    def _controller_debug_cb(self, msg: Float64MultiArray) -> None:
        if len(msg.data) < 20:
            return
        self.controller_debug = [float(value) for value in msg.data[:20]]

    def _flush_row(self) -> None:
        if (
            self.latest_status is None or
            not self.carrier_odom_received or
            not self.mini_odom_received
        ):
            self._check_timeout()
            return

        ros_time = (
            self.latest_status.header.stamp.sec +
            self.latest_status.header.stamp.nanosec * 1e-9
        )
        t = 0.0 if self.start_ros_time is None else ros_time - self.start_ros_time
        measured_rel = [
            self.mini_position[0] - self.carrier_position[0],
            self.mini_position[1] - self.carrier_position[1],
            self.mini_position[2] - self.carrier_position[2],
        ]
        measured_rel_distance = sum(value * value for value in measured_rel) ** 0.5
        measured_rel_velocity = [
            self.mini_velocity[0] - self.carrier_velocity[0],
            self.mini_velocity[1] - self.carrier_velocity[1],
            self.mini_velocity[2] - self.carrier_velocity[2],
        ]
        measured_rel_speed = sum(value * value for value in measured_rel_velocity) ** 0.5
        if self.mini_px4_local_z_global_valid > 0.5 and math.isfinite(self.mini_px4_local_ref_alt):
            mini_px4_local_alt_from_ref = -self.mini_px4_local_z + self.mini_px4_local_ref_alt
        else:
            mini_px4_local_alt_from_ref = math.nan
        if math.isfinite(self.mini_px4_global_alt) and math.isfinite(mini_px4_local_alt_from_ref):
            mini_px4_alt_source_delta = self.mini_px4_global_alt - mini_px4_local_alt_from_ref
        else:
            mini_px4_alt_source_delta = math.nan
        self.writer.writerow([
            f"{t:.3f}",
            self.latest_status.phase,
            f"{self.latest_status.relative_distance:.6f}",
            f"{self.latest_status.relative_position.x:.6f}",
            f"{self.latest_status.relative_position.y:.6f}",
            f"{self.latest_status.relative_position.z:.6f}",
            f"{self.latest_status.relative_velocity.x:.6f}",
            f"{self.latest_status.relative_velocity.y:.6f}",
            f"{self.latest_status.relative_velocity.z:.6f}",
            f"{self.carrier_position[0]:.6f}",
            f"{self.carrier_position[1]:.6f}",
            f"{self.carrier_position[2]:.6f}",
            f"{self.carrier_velocity[0]:.6f}",
            f"{self.carrier_velocity[1]:.6f}",
            f"{self.carrier_velocity[2]:.6f}",
            f"{self.carrier_euler_deg[0]:.6f}",
            f"{self.carrier_euler_deg[1]:.6f}",
            f"{self.carrier_euler_deg[2]:.6f}",
            f"{self.mini_position[0]:.6f}",
            f"{self.mini_position[1]:.6f}",
            f"{self.mini_position[2]:.6f}",
            f"{self.mini_velocity[0]:.6f}",
            f"{self.mini_velocity[1]:.6f}",
            f"{self.mini_velocity[2]:.6f}",
            f"{self.mini_euler_deg[0]:.6f}",
            f"{self.mini_euler_deg[1]:.6f}",
            f"{self.mini_euler_deg[2]:.6f}",
            f"{self.carrier_setpoint[0]:.6f}",
            f"{self.carrier_setpoint[1]:.6f}",
            f"{self.carrier_setpoint[2]:.6f}",
            f"{self.carrier_cmd_velocity[0]:.6f}",
            f"{self.carrier_cmd_velocity[1]:.6f}",
            f"{self.carrier_cmd_velocity[2]:.6f}",
            f"{self.mini_target[0]:.6f}",
            f"{self.mini_target[1]:.6f}",
            f"{self.mini_target[2]:.6f}",
            f"{measured_rel[0]:.6f}",
            f"{measured_rel[1]:.6f}",
            f"{measured_rel[2]:.6f}",
            f"{measured_rel_distance:.6f}",
            f"{measured_rel_velocity[0]:.6f}",
            f"{measured_rel_velocity[1]:.6f}",
            f"{measured_rel_velocity[2]:.6f}",
            f"{measured_rel_speed:.6f}",
            f"{self.mini_px4_true_airspeed:.6f}",
            f"{self.mini_px4_throttle_filtered:.6f}",
            f"{self.mini_px4_global_alt:.6f}",
            f"{self.mini_px4_global_alt_valid:.0f}",
            f"{self.mini_px4_local_z:.6f}",
            f"{self.mini_px4_local_vz:.6f}",
            f"{self.mini_px4_local_ref_alt:.6f}",
            f"{self.mini_px4_local_z_global_valid:.0f}",
            f"{mini_px4_local_alt_from_ref:.6f}",
            f"{mini_px4_alt_source_delta:.6f}",
            f"{self.mini_tecs_altitude_sp:.6f}",
            f"{self.mini_tecs_altitude_reference:.6f}",
            f"{self.mini_tecs_height_rate_reference:.6f}",
            f"{self.mini_tecs_height_rate_sp:.6f}",
            f"{self.mini_tecs_height_rate:.6f}",
            f"{self.mini_tecs_true_airspeed_sp:.6f}",
            f"{self.mini_tecs_true_airspeed_filtered:.6f}",
            f"{self.mini_tecs_throttle_sp:.6f}",
            f"{self.mini_tecs_throttle_trim:.6f}",
            f"{self.mini_tecs_pitch_sp_deg:.6f}",
            f"{self.mini_tecs_underspeed_ratio:.6f}",
            f"{self.mini_tecs_fast_descend_ratio:.6f}",
            f"{self.mini_tecs_total_energy_rate_sp:.6f}",
            f"{self.mini_tecs_total_energy_rate:.6f}",
            f"{self.mini_fw_altitude_sp:.6f}",
            f"{self.mini_fw_height_rate_sp:.6f}",
            f"{self.mini_fw_equivalent_airspeed_sp:.6f}",
            f"{self.mini_fw_pitch_direct_deg:.6f}",
            f"{self.mini_fw_throttle_direct:.6f}",
            f"{self.mini_attitude_sp_thrust_x:.6f}",
            f"{self.mini_terminal_sync_debug[0]:.6f}",
            f"{self.mini_terminal_sync_debug[1]:.6f}",
            f"{self.mini_terminal_sync_debug[2]:.6f}",
            f"{self.mini_terminal_sync_debug[3]:.6f}",
            f"{self.mini_terminal_sync_debug[4]:.6f}",
            f"{self.mini_terminal_sync_debug[5]:.6f}",
            f"{self.mini_terminal_sync_debug[6]:.6f}",
            f"{self.mini_terminal_sync_debug[7]:.6f}",
            f"{self.mini_terminal_sync_debug[8]:.6f}",
            f"{self.mini_terminal_sync_debug[9]:.6f}",
            f"{self.mini_terminal_sync_debug[10]:.6f}",
            f"{self.mini_terminal_sync_debug[11]:.6f}",
            f"{self.mini_terminal_sync_debug[12]:.6f}",
            f"{self.mini_terminal_sync_debug[13]:.6f}",
            f"{self.mini_terminal_sync_debug[14]:.6f}",
            f"{self.mini_terminal_sync_debug[15]:.6f}",
            f"{self.mini_handoff_debug[0]:.6f}",
            f"{self.mini_handoff_debug[1]:.6f}",
            f"{self.mini_handoff_debug[2]:.6f}",
            f"{self.mini_handoff_debug[3]:.6f}",
            f"{self.mini_handoff_debug[4]:.6f}",
            f"{self.mini_handoff_debug[5]:.6f}",
            f"{self.mini_energy_debug[0]:.6f}",
            f"{self.mini_energy_debug[1]:.6f}",
            f"{self.mini_energy_debug[2]:.6f}",
            f"{self.mini_energy_debug[3]:.6f}",
            f"{self.mini_energy_debug[4]:.6f}",
            f"{self.mini_energy_debug[5]:.6f}",
            f"{self.controller_debug[0]:.6f}",
            f"{self.controller_debug[1]:.6f}",
            f"{self.controller_debug[2]:.6f}",
            f"{self.controller_debug[3]:.6f}",
            f"{self.controller_debug[4]:.6f}",
            f"{self.controller_debug[5]:.6f}",
            f"{self.controller_debug[6]:.6f}",
            f"{self.controller_debug[7]:.6f}",
            f"{self.controller_debug[8]:.6f}",
            f"{self.controller_debug[9]:.6f}",
            f"{self.controller_debug[10]:.6f}",
            f"{self.controller_debug[11]:.6f}",
            f"{self.controller_debug[12]:.6f}",
            f"{self.controller_debug[13]:.6f}",
            f"{self.controller_debug[14]:.6f}",
            f"{self.controller_debug[15]:.6f}",
            f"{self.controller_debug[16]:.6f}",
            f"{self.controller_debug[17]:.6f}",
            f"{self.controller_debug[18]:.6f}",
            f"{self.controller_debug[19]:.6f}",
        ])
        self.rows_written += 1

        if self.latest_status.phase == "COMPLETED":
            if self.completed_wall_time is None:
                self.completed_wall_time = time.time()
                self.completed_rows_written = 0
            self.completed_rows_written += 1
            elapsed_after_completed = time.time() - self.completed_wall_time
            if (
                elapsed_after_completed >= self.post_completed_settle_sec and
                self.completed_rows_written >= self.post_completed_rows
            ):
                self._finalize("completed")
                return
        else:
            sync_score = self.mini_terminal_sync_debug[2]
            line_lateral_error = self.mini_terminal_sync_debug[10]
            height_error = self.mini_terminal_sync_debug[11]
            prototype_capture_ready = (
                self.latest_status.phase == "DOCKING" and
                measured_rel_distance <= self.prototype_capture_distance_m and
                measured_rel_speed <= self.prototype_capture_rel_speed_mps and
                math.isfinite(sync_score) and
                sync_score >= self.prototype_capture_sync_score and
                math.isfinite(line_lateral_error) and
                abs(line_lateral_error) <= self.prototype_capture_max_line_lateral_error_m and
                math.isfinite(height_error) and
                abs(height_error) <= self.prototype_capture_max_height_error_m
            )
            if prototype_capture_ready:
                self.prototype_capture_rows += 1
            else:
                self.prototype_capture_rows = 0

            if self.prototype_capture_rows >= self.prototype_capture_hold_rows:
                self._finalize("prototype_captured")
                return

        self._check_timeout()

    def _check_timeout(self) -> None:
        if time.time() - self.start_wall >= self.duration_sec:
            self._finalize("timeout")

    def _finalize(self, reason: str) -> None:
        if self.csv_file.closed:
            return
        self.csv_file.flush()
        self.csv_file.close()
        with self.metadata_path.open("w", encoding="utf-8") as file:
            file.write(f"reason={reason}\n")
            file.write(f"rows={self.rows_written}\n")
            file.write(f"phase={self.latest_phase}\n")
            file.write(f"completed_rows={self.completed_rows_written}\n")
            file.write(
                f"post_completed_settle_sec={self.post_completed_settle_sec:.3f}\n"
            )
            file.write(f"post_completed_rows={self.post_completed_rows}\n")
            file.write(f"csv={self.csv_path}\n")
        self.get_logger().info(f"Experiment finished: {reason}, rows={self.rows_written}")
        raise SystemExit(0)


def main() -> None:
    rclpy.init()
    node = ExperimentLogger()
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    finally:
        if not node.csv_file.closed:
            node.csv_file.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
