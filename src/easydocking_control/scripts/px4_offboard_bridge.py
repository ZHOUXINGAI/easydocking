#!/usr/bin/env python3

import math

import rclpy
from easydocking_msgs.msg import DockingCommand
from easydocking_msgs.msg import DockingStatus
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand
except ImportError:  # pragma: no cover
    OffboardControlMode = None
    TrajectorySetpoint = None
    VehicleCommand = None


class Px4OffboardBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_offboard_bridge")
        self.declare_parameter("uav_name", "mini")
        self.declare_parameter("px4_namespace", "/px4_2")
        self.declare_parameter("vehicle_id", 2)
        self.declare_parameter("arm_on_start", False)
        self.declare_parameter("world_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("use_velocity_feedforward", True)
        self.declare_parameter("use_position_setpoint", True)
        self.declare_parameter("activate_on_launch", False)
        self.declare_parameter("same_direction_guard_enabled", False)
        self.declare_parameter("same_direction_guard_only_carrier", True)
        self.declare_parameter("same_direction_guard_min_mini_speed_mps", 4.0)
        self.declare_parameter("same_direction_guard_reverse_threshold_mps", 0.1)
        self.declare_parameter("same_direction_guard_release_forward_mps", 0.8)
        self.declare_parameter("same_direction_guard_min_forward_command_mps", 1.2)
        self.declare_parameter("same_direction_guard_force_min_forward_command_mps", 3.0)
        self.declare_parameter("same_direction_guard_max_lateral_command_mps", 1.8)
        self.declare_parameter("same_direction_guard_force_duration_sec", 8.0)
        self.declare_parameter("same_direction_guard_log_period_sec", 1.0)
        self.declare_parameter("same_direction_guard_predictive_lead_enabled", True)
        self.declare_parameter("same_direction_guard_predictive_adaptive_horizon_enabled", True)
        self.declare_parameter("same_direction_guard_predictive_min_horizon_sec", 2.0)
        self.declare_parameter("same_direction_guard_predictive_horizon_sec", 8.0)
        self.declare_parameter("same_direction_guard_predictive_horizon_gain", 0.85)
        self.declare_parameter("same_direction_guard_predictive_use_acceleration", True)
        self.declare_parameter("same_direction_guard_predictive_max_accel_mps2", 3.0)
        self.declare_parameter("same_direction_guard_predictive_accel_filter_alpha", 0.35)
        self.declare_parameter("same_direction_guard_predictive_tangent_weight", 0.25)

        self.uav_name = self.get_parameter("uav_name").value
        self.px4_namespace = self.get_parameter("px4_namespace").value.rstrip("/")
        self.vehicle_id = int(self.get_parameter("vehicle_id").value)
        self.arm_on_start = bool(self.get_parameter("arm_on_start").value)
        self.use_velocity_feedforward = bool(self.get_parameter("use_velocity_feedforward").value)
        self.use_position_setpoint = bool(self.get_parameter("use_position_setpoint").value)
        self.activate_on_launch = bool(self.get_parameter("activate_on_launch").value)
        self.same_direction_guard_enabled = bool(
            self.get_parameter("same_direction_guard_enabled").value
        )
        self.same_direction_guard_only_carrier = bool(
            self.get_parameter("same_direction_guard_only_carrier").value
        )
        self.same_direction_guard_min_mini_speed_mps = float(
            self.get_parameter("same_direction_guard_min_mini_speed_mps").value
        )
        self.same_direction_guard_reverse_threshold_mps = float(
            self.get_parameter("same_direction_guard_reverse_threshold_mps").value
        )
        self.same_direction_guard_release_forward_mps = float(
            self.get_parameter("same_direction_guard_release_forward_mps").value
        )
        self.same_direction_guard_min_forward_command_mps = float(
            self.get_parameter("same_direction_guard_min_forward_command_mps").value
        )
        self.same_direction_guard_force_min_forward_command_mps = float(
            self.get_parameter("same_direction_guard_force_min_forward_command_mps").value
        )
        self.same_direction_guard_max_lateral_command_mps = float(
            self.get_parameter("same_direction_guard_max_lateral_command_mps").value
        )
        self.same_direction_guard_force_duration_sec = max(
            0.0, float(self.get_parameter("same_direction_guard_force_duration_sec").value)
        )
        self.same_direction_guard_log_period_sec = max(
            0.2, float(self.get_parameter("same_direction_guard_log_period_sec").value)
        )
        self.same_direction_guard_predictive_lead_enabled = bool(
            self.get_parameter("same_direction_guard_predictive_lead_enabled").value
        )
        self.same_direction_guard_predictive_adaptive_horizon_enabled = bool(
            self.get_parameter("same_direction_guard_predictive_adaptive_horizon_enabled").value
        )
        self.same_direction_guard_predictive_min_horizon_sec = max(
            0.0, float(self.get_parameter("same_direction_guard_predictive_min_horizon_sec").value)
        )
        self.same_direction_guard_predictive_horizon_sec = max(
            0.0, float(self.get_parameter("same_direction_guard_predictive_horizon_sec").value)
        )
        self.same_direction_guard_predictive_horizon_gain = max(
            0.0, float(self.get_parameter("same_direction_guard_predictive_horizon_gain").value)
        )
        self.same_direction_guard_predictive_use_acceleration = bool(
            self.get_parameter("same_direction_guard_predictive_use_acceleration").value
        )
        self.same_direction_guard_predictive_max_accel_mps2 = max(
            0.0, float(self.get_parameter("same_direction_guard_predictive_max_accel_mps2").value)
        )
        self.same_direction_guard_predictive_accel_filter_alpha = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter("same_direction_guard_predictive_accel_filter_alpha").value),
            ),
        )
        self.same_direction_guard_predictive_tangent_weight = max(
            0.0,
            min(
                1.0,
                float(self.get_parameter("same_direction_guard_predictive_tangent_weight").value),
            ),
        )
        self.world_offset = [
            float(value) for value in self.get_parameter("world_offset").value
        ]

        self.pose_setpoint = None
        self.velocity_setpoint = None
        self.odom = None
        self.mini_odom = None
        self.offboard_counter = 0
        self.offboard_active = self.activate_on_launch
        self.mode_sent = False
        self.arm_sent = False
        self.same_direction_guard_active = False
        self.same_direction_guard_last_log_time_sec = -1e9
        self.offboard_start_time_sec = None
        self.controller_phase = "IDLE"
        self.mini_last_odom_time_sec = None
        self.mini_last_velocity_xy = None
        self.mini_accel_xy = (0.0, 0.0)

        self.create_subscription(
            PoseStamped, f"/{self.uav_name}/setpoint/pose", self._pose_setpoint_cb, 10
        )
        self.create_subscription(
            TwistStamped, f"/{self.uav_name}/setpoint/velocity", self._velocity_setpoint_cb, 10
        )
        self.create_subscription(Odometry, f"/{self.uav_name}/odom", self._odom_cb, 10)
        if (
            self.same_direction_guard_enabled and
            (not self.same_direction_guard_only_carrier or self.uav_name == "carrier")
        ):
            self.create_subscription(Odometry, "/mini/odom", self._mini_odom_cb, 10)
        self.create_subscription(DockingCommand, "/docking/command", self._command_cb, 10)
        self.create_subscription(
            DockingCommand,
            "/docking/command_latched",
            self._latched_command_cb,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.create_subscription(DockingStatus, "/docking/status", self._status_cb, 10)

        if OffboardControlMode is None:
            self.get_logger().error(
                "px4_msgs is not installed. Clone/build PX4 px4_msgs first, then rerun this node."
            )
            return

        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.offboard_mode_pub = self.create_publisher(
            OffboardControlMode,
            f"{self.px4_namespace}/fmu/in/offboard_control_mode",
            px4_qos,
        )
        self.trajectory_pub = self.create_publisher(
            TrajectorySetpoint,
            f"{self.px4_namespace}/fmu/in/trajectory_setpoint",
            px4_qos,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            f"{self.px4_namespace}/fmu/in/vehicle_command",
            px4_qos,
        )
        self.timer = self.create_timer(0.05, self._timer_cb)

    def _pose_setpoint_cb(self, msg: PoseStamped) -> None:
        self.pose_setpoint = msg

    def _velocity_setpoint_cb(self, msg: TwistStamped) -> None:
        self.velocity_setpoint = msg

    def _odom_cb(self, msg: Odometry) -> None:
        self.odom = msg

    def _mini_odom_cb(self, msg: Odometry) -> None:
        sample_time_sec = self._msg_time_sec(msg)
        mini_vx = float(msg.twist.twist.linear.x)
        mini_vy = float(msg.twist.twist.linear.y)
        if self.mini_last_odom_time_sec is not None and self.mini_last_velocity_xy is not None:
            dt = sample_time_sec - self.mini_last_odom_time_sec
            if 0.01 <= dt <= 1.0:
                raw_ax = (mini_vx - self.mini_last_velocity_xy[0]) / dt
                raw_ay = (mini_vy - self.mini_last_velocity_xy[1]) / dt
                raw_norm = math.hypot(raw_ax, raw_ay)
                if raw_norm > self.same_direction_guard_predictive_max_accel_mps2 and raw_norm > 1e-6:
                    scale = self.same_direction_guard_predictive_max_accel_mps2 / raw_norm
                    raw_ax *= scale
                    raw_ay *= scale
                alpha = self.same_direction_guard_predictive_accel_filter_alpha
                self.mini_accel_xy = (
                    (1.0 - alpha) * self.mini_accel_xy[0] + alpha * raw_ax,
                    (1.0 - alpha) * self.mini_accel_xy[1] + alpha * raw_ay,
                )
        self.mini_last_odom_time_sec = sample_time_sec
        self.mini_last_velocity_xy = (mini_vx, mini_vy)
        self.mini_odom = msg

    def _command_cb(self, msg: DockingCommand) -> None:
        self._handle_command(msg, source="direct")

    def _latched_command_cb(self, msg: DockingCommand) -> None:
        self._handle_command(msg, source="latched")

    def _handle_command(self, msg: DockingCommand, source: str) -> None:
        command = msg.command.strip().upper()
        if command == "START":
            was_active = self.offboard_active
            self.offboard_active = True
            self.offboard_counter = 0
            self.mode_sent = False
            self.arm_sent = False
            self.offboard_start_time_sec = self.get_clock().now().nanoseconds / 1e9
            if was_active:
                self.get_logger().info(
                    f"{self.uav_name}: received START via {source}, refreshing PX4 offboard/arm requests"
                )
            else:
                self.get_logger().info(
                    f"{self.uav_name}: received START via {source}, enabling PX4 offboard stream"
                )
        elif command in {"STOP", "RESET"}:
            self.offboard_active = self.activate_on_launch
            self.offboard_counter = 0
            self.mode_sent = False
            self.arm_sent = False
            self.offboard_start_time_sec = None
            self.get_logger().info(
                f"{self.uav_name}: received {command} via {source}, disabling PX4 offboard stream"
            )

    def _status_cb(self, msg: DockingStatus) -> None:
        self.controller_phase = str(msg.phase).strip().upper()
        if msg.is_active and not self.offboard_active:
            self.offboard_active = True
            self.offboard_counter = 0
            self.mode_sent = False
            self.arm_sent = False
            self.offboard_start_time_sec = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f"{self.uav_name}: controller is active ({msg.phase}), enabling PX4 offboard stream"
            )
        elif not msg.is_active and self.offboard_active and not self.activate_on_launch:
            self.offboard_active = False
            self.offboard_counter = 0
            self.mode_sent = False
            self.arm_sent = False
            self.offboard_start_time_sec = None
            self.get_logger().info(
                f"{self.uav_name}: controller is inactive ({msg.phase}), disabling PX4 offboard stream"
            )

    def _timer_cb(self) -> None:
        if not self.offboard_active:
            return

        if self.pose_setpoint is None and self.velocity_setpoint is None:
            return

        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        use_position = self.use_position_setpoint and self.pose_setpoint is not None
        use_velocity = self.use_velocity_feedforward and self.velocity_setpoint is not None
        if not use_position and not use_velocity:
            return

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = bool(use_position)
        offboard_mode.velocity = bool(use_velocity)
        offboard_mode.acceleration = False
        offboard_mode.attitude = False
        offboard_mode.body_rate = False
        if hasattr(offboard_mode, "thrust_and_torque"):
            offboard_mode.thrust_and_torque = False
        if hasattr(offboard_mode, "direct_actuator"):
            offboard_mode.direct_actuator = False
        self.offboard_mode_pub.publish(offboard_mode)

        trajectory = TrajectorySetpoint()
        trajectory.timestamp = timestamp
        trajectory.position = [math.nan, math.nan, math.nan]
        trajectory.velocity = [math.nan, math.nan, math.nan]
        trajectory.acceleration = [math.nan, math.nan, math.nan]
        trajectory.yaw = math.nan
        trajectory.yawspeed = math.nan

        if use_position and self.pose_setpoint is not None:
            trajectory.position = self._world_position_to_px4_local(self.pose_setpoint)

        if use_velocity and self.velocity_setpoint is not None:
            trajectory.velocity = self._world_velocity_to_px4_local(self.velocity_setpoint)
        else:
            trajectory.velocity = [math.nan, math.nan, math.nan]

        use_position, use_velocity, trajectory = self._apply_same_direction_guard(
            use_position,
            use_velocity,
            trajectory,
        )
        offboard_mode.position = bool(use_position)
        offboard_mode.velocity = bool(use_velocity)

        self.trajectory_pub.publish(trajectory)
        self.offboard_counter += 1

        if self.offboard_counter >= 20 and not self.mode_sent:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.mode_sent = True
            self.get_logger().info(f"{self.uav_name}: requested OFFBOARD mode")
        if self.mode_sent and self.arm_on_start and not self.arm_sent:
            self._publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.arm_sent = True
            self.get_logger().info(f"{self.uav_name}: requested ARM")

    def _publish_vehicle_command(self, command: int, param1: float = 0.0, param2: float = 0.0) -> None:
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = self.vehicle_id
        msg.target_component = 1
        msg.source_system = self.vehicle_id
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def _world_position_to_px4_local(self, msg: PoseStamped) -> list[float]:
        x = float(msg.pose.position.x) - self.world_offset[0]
        y = float(msg.pose.position.y) - self.world_offset[1]
        z = float(msg.pose.position.z) - self.world_offset[2]
        return [x, y, -z]

    def _world_velocity_to_px4_local(self, msg: TwistStamped) -> list[float]:
        return [
            float(msg.twist.linear.x),
            float(msg.twist.linear.y),
            -float(msg.twist.linear.z),
        ]

    def _msg_time_sec(self, msg: Odometry) -> float:
        stamp = msg.header.stamp
        stamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1e-9
        if stamp_sec > 0.0:
            return stamp_sec
        return self.get_clock().now().nanoseconds / 1e9

    def _apply_same_direction_guard(
        self,
        use_position: bool,
        use_velocity: bool,
        trajectory: TrajectorySetpoint,
    ) -> tuple[bool, bool, TrajectorySetpoint]:
        if not self.same_direction_guard_enabled:
            self.same_direction_guard_active = False
            return use_position, use_velocity, trajectory
        if self.same_direction_guard_only_carrier and self.uav_name != "carrier":
            self.same_direction_guard_active = False
            return use_position, use_velocity, trajectory
        if self.odom is None or self.mini_odom is None:
            return use_position, use_velocity, trajectory
        if self.controller_phase in {"", "IDLE"}:
            self.same_direction_guard_active = False
            return use_position, use_velocity, trajectory

        mini_vx = float(self.mini_odom.twist.twist.linear.x)
        mini_vy = float(self.mini_odom.twist.twist.linear.y)
        mini_speed = math.hypot(mini_vx, mini_vy)
        if mini_speed < self.same_direction_guard_min_mini_speed_mps:
            if self.same_direction_guard_active:
                self.same_direction_guard_active = False
            return use_position, use_velocity, trajectory

        mini_dir_x = mini_vx / mini_speed
        mini_dir_y = mini_vy / mini_speed

        now_sec = self.get_clock().now().nanoseconds / 1e9
        phase_force_guard = self.controller_phase in {"APPROACH"}
        force_guard = (
            phase_force_guard or
            self.offboard_start_time_sec is not None and
            (now_sec - self.offboard_start_time_sec) <= self.same_direction_guard_force_duration_sec
        )

        guard_dir_x = mini_dir_x
        guard_dir_y = mini_dir_y
        predictive_horizon = self.same_direction_guard_predictive_horizon_sec
        if self.same_direction_guard_predictive_lead_enabled and force_guard:
            mini_px = float(self.mini_odom.pose.pose.position.x)
            mini_py = float(self.mini_odom.pose.pose.position.y)
            carrier_px = float(self.odom.pose.pose.position.x)
            carrier_py = float(self.odom.pose.pose.position.y)
            if self.same_direction_guard_predictive_adaptive_horizon_enabled:
                rel_dist = math.hypot(mini_px - carrier_px, mini_py - carrier_py)
                speed_ref = max(
                    mini_speed,
                    self.same_direction_guard_force_min_forward_command_mps,
                    self.same_direction_guard_min_forward_command_mps,
                    1.0,
                )
                adaptive_horizon = self.same_direction_guard_predictive_horizon_gain * rel_dist / speed_ref
                predictive_horizon = max(
                    self.same_direction_guard_predictive_min_horizon_sec,
                    min(self.same_direction_guard_predictive_horizon_sec, adaptive_horizon),
                )
            mini_ax = self.mini_accel_xy[0] if self.same_direction_guard_predictive_use_acceleration else 0.0
            mini_ay = self.mini_accel_xy[1] if self.same_direction_guard_predictive_use_acceleration else 0.0
            mini_pred_x = mini_px + mini_vx * predictive_horizon + 0.5 * mini_ax * predictive_horizon ** 2
            mini_pred_y = mini_py + mini_vy * predictive_horizon + 0.5 * mini_ay * predictive_horizon ** 2
            mini_pred_vx = mini_vx + mini_ax * predictive_horizon
            mini_pred_vy = mini_vy + mini_ay * predictive_horizon
            mini_pred_speed = math.hypot(mini_pred_vx, mini_pred_vy)
            if mini_pred_speed > 1e-6:
                mini_pred_dir_x = mini_pred_vx / mini_pred_speed
                mini_pred_dir_y = mini_pred_vy / mini_pred_speed
            else:
                mini_pred_dir_x = mini_dir_x
                mini_pred_dir_y = mini_dir_y
            lead_vec_x = mini_pred_x - carrier_px
            lead_vec_y = mini_pred_y - carrier_py
            lead_norm = math.hypot(lead_vec_x, lead_vec_y)
            if lead_norm > 1e-6:
                lead_vec_x /= lead_norm
                lead_vec_y /= lead_norm
                tangent_weight = self.same_direction_guard_predictive_tangent_weight
                blend_x = tangent_weight * mini_pred_dir_x + (1.0 - tangent_weight) * lead_vec_x
                blend_y = tangent_weight * mini_pred_dir_y + (1.0 - tangent_weight) * lead_vec_y
                blend_norm = math.hypot(blend_x, blend_y)
                if blend_norm > 1e-6:
                    candidate_dir_x = blend_x / blend_norm
                    candidate_dir_y = blend_y / blend_norm
                    if (
                        candidate_dir_x * mini_pred_dir_x +
                        candidate_dir_y * mini_pred_dir_y
                    ) < 0.0:
                        guard_dir_x = mini_pred_dir_x
                        guard_dir_y = mini_pred_dir_y
                    else:
                        guard_dir_x = candidate_dir_x
                        guard_dir_y = candidate_dir_y

        guard_lat_x = -guard_dir_y
        guard_lat_y = guard_dir_x
        carrier_vx = float(self.odom.twist.twist.linear.x)
        carrier_vy = float(self.odom.twist.twist.linear.y)
        carrier_forward_actual = carrier_vx * guard_dir_x + carrier_vy * guard_dir_y

        if force_guard:
            self.same_direction_guard_active = True
        elif self.same_direction_guard_active:
            if carrier_forward_actual >= self.same_direction_guard_release_forward_mps:
                self.same_direction_guard_active = False
        else:
            if carrier_forward_actual <= -self.same_direction_guard_reverse_threshold_mps:
                self.same_direction_guard_active = True

        if not self.same_direction_guard_active:
            return use_position, use_velocity, trajectory

        cmd_vx = float(trajectory.velocity[0]) if math.isfinite(trajectory.velocity[0]) else 0.0
        cmd_vy = float(trajectory.velocity[1]) if math.isfinite(trajectory.velocity[1]) else 0.0
        cmd_vz = -float(trajectory.velocity[2]) if math.isfinite(trajectory.velocity[2]) else 0.0
        cmd_forward = cmd_vx * guard_dir_x + cmd_vy * guard_dir_y
        cmd_lateral = cmd_vx * guard_lat_x + cmd_vy * guard_lat_y
        min_forward_command = self.same_direction_guard_min_forward_command_mps
        if force_guard:
            min_forward_command = max(
                min_forward_command,
                self.same_direction_guard_force_min_forward_command_mps,
            )
        cmd_forward = max(cmd_forward, min_forward_command)
        cmd_lateral = max(
            -self.same_direction_guard_max_lateral_command_mps,
            min(self.same_direction_guard_max_lateral_command_mps, cmd_lateral),
        )
        cmd_vx = cmd_forward * guard_dir_x + cmd_lateral * guard_lat_x
        cmd_vy = cmd_forward * guard_dir_y + cmd_lateral * guard_lat_y
        trajectory.velocity = [cmd_vx, cmd_vy, -cmd_vz]
        trajectory.position = [math.nan, math.nan, math.nan]
        use_position = False
        use_velocity = True

        if now_sec - self.same_direction_guard_last_log_time_sec >= self.same_direction_guard_log_period_sec:
            self.same_direction_guard_last_log_time_sec = now_sec
            self.get_logger().info(
                f"{self.uav_name}: reverse-guard active "
                f"mini_speed={mini_speed:.2f} carrier_forward={carrier_forward_actual:.2f} "
                f"cmd_forward={cmd_forward:.2f} cmd_lateral={cmd_lateral:.2f} "
                f"guard_dir=({guard_dir_x:.2f},{guard_dir_y:.2f}) "
                f"horizon={predictive_horizon:.2f} force={force_guard}"
            )
        return use_position, use_velocity, trajectory


def main() -> None:
    rclpy.init()
    node = Px4OffboardBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
