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
        self.declare_parameter("activate_on_launch", False)

        self.uav_name = self.get_parameter("uav_name").value
        self.px4_namespace = self.get_parameter("px4_namespace").value.rstrip("/")
        self.vehicle_id = int(self.get_parameter("vehicle_id").value)
        self.arm_on_start = bool(self.get_parameter("arm_on_start").value)
        self.use_velocity_feedforward = bool(self.get_parameter("use_velocity_feedforward").value)
        self.activate_on_launch = bool(self.get_parameter("activate_on_launch").value)
        self.world_offset = [
            float(value) for value in self.get_parameter("world_offset").value
        ]

        self.pose_setpoint = None
        self.velocity_setpoint = None
        self.odom = None
        self.offboard_counter = 0
        self.offboard_active = self.activate_on_launch
        self.mode_sent = False
        self.arm_sent = False

        self.create_subscription(
            PoseStamped, f"/{self.uav_name}/setpoint/pose", self._pose_setpoint_cb, 10
        )
        self.create_subscription(
            TwistStamped, f"/{self.uav_name}/setpoint/velocity", self._velocity_setpoint_cb, 10
        )
        self.create_subscription(Odometry, f"/{self.uav_name}/odom", self._odom_cb, 10)
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
            self.get_logger().info(
                f"{self.uav_name}: received {command} via {source}, disabling PX4 offboard stream"
            )

    def _status_cb(self, msg: DockingStatus) -> None:
        if msg.is_active and not self.offboard_active:
            self.offboard_active = True
            self.offboard_counter = 0
            self.mode_sent = False
            self.arm_sent = False
            self.get_logger().info(
                f"{self.uav_name}: controller is active ({msg.phase}), enabling PX4 offboard stream"
            )
        elif not msg.is_active and self.offboard_active and not self.activate_on_launch:
            self.offboard_active = False
            self.offboard_counter = 0
            self.mode_sent = False
            self.arm_sent = False
            self.get_logger().info(
                f"{self.uav_name}: controller is inactive ({msg.phase}), disabling PX4 offboard stream"
            )

    def _timer_cb(self) -> None:
        if not self.offboard_active:
            return

        if self.pose_setpoint is None:
            return

        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        offboard_mode.velocity = self.use_velocity_feedforward and self.velocity_setpoint is not None
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

        if self.pose_setpoint is not None:
            trajectory.position = self._world_position_to_px4_local(self.pose_setpoint)

        if self.use_velocity_feedforward and self.velocity_setpoint is not None:
            trajectory.velocity = self._world_velocity_to_px4_local(self.velocity_setpoint)
        else:
            trajectory.velocity = [math.nan, math.nan, math.nan]

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


def main() -> None:
    rclpy.init()
    node = Px4OffboardBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
