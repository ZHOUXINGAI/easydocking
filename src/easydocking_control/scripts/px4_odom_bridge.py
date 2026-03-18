#!/usr/bin/env python3

import rclpy
from easydocking_msgs.msg import DockingStatus
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

try:
    from px4_msgs.msg import VehicleLocalPosition, VehicleOdometry, VehicleStatus
except ImportError:  # pragma: no cover
    VehicleLocalPosition = None
    VehicleOdometry = None
    VehicleStatus = None


class Px4OdomBridge(Node):
    def __init__(self) -> None:
        super().__init__("px4_odom_bridge")
        self.declare_parameter("uav_name", "mini")
        self.declare_parameter("px4_namespace", "/px4_2")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("world_offset", [0.0, 0.0, 0.0])
        self.declare_parameter("attach_relative_position", [0.0, 0.0, 0.8])

        if VehicleOdometry is None or VehicleStatus is None or VehicleLocalPosition is None:
            self.get_logger().error("px4_msgs is not installed, cannot bridge odometry.")
            return

        self.uav_name = self.get_parameter("uav_name").value
        self.px4_namespace = self.get_parameter("px4_namespace").value.rstrip("/")
        self.world_frame = self.get_parameter("world_frame").value
        self.world_offset = [
            float(value) for value in self.get_parameter("world_offset").value
        ]
        self.attach_relative_position = [
            float(value) for value in self.get_parameter("attach_relative_position").value
        ]
        self.controller_active = False
        self.current_phase = "IDLE"
        self.arming_state = VehicleStatus.ARMING_STATE_DISARMED
        self.frozen_odom = None
        self.carrier_odom = None
        self.local_position = None

        self.publisher = self.create_publisher(Odometry, f"/{self.uav_name}/odom", 10)
        px4_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            VehicleOdometry,
            f"{self.px4_namespace}/fmu/out/vehicle_odometry",
            self._odom_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleStatus,
            f"{self.px4_namespace}/fmu/out/vehicle_status_v2",
            self._vehicle_status_cb,
            px4_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            f"{self.px4_namespace}/fmu/out/vehicle_local_position_v1",
            self._local_position_cb,
            px4_qos,
        )
        self.create_subscription(
            DockingStatus,
            "/docking/status",
            self._status_cb,
            10,
        )
        if self.uav_name == "mini":
            self.create_subscription(
                Odometry,
                "/carrier/odom",
                self._carrier_odom_cb,
                10,
            )

    def _vehicle_status_cb(self, msg: VehicleStatus) -> None:
        self.arming_state = int(msg.arming_state)

    def _status_cb(self, msg: DockingStatus) -> None:
        self.controller_active = bool(msg.is_active)
        self.current_phase = msg.phase.upper()

    def _carrier_odom_cb(self, msg: Odometry) -> None:
        self.carrier_odom = msg

    def _local_position_cb(self, msg: VehicleLocalPosition) -> None:
        self.local_position = msg

    def _build_attached_odom(self, stamp) -> Odometry:
        assert self.carrier_odom is not None
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = f"{self.uav_name}/base_link"
        odom.pose.pose.position.x = (
            self.carrier_odom.pose.pose.position.x + self.attach_relative_position[0]
        )
        odom.pose.pose.position.y = (
            self.carrier_odom.pose.pose.position.y + self.attach_relative_position[1]
        )
        odom.pose.pose.position.z = (
            self.carrier_odom.pose.pose.position.z + self.attach_relative_position[2]
        )
        odom.pose.pose.orientation = self.carrier_odom.pose.pose.orientation
        odom.twist = self.carrier_odom.twist
        return odom

    def _odom_cb(self, msg: VehicleOdometry) -> None:
        if (
            self.uav_name == "mini" and
            self.current_phase == "COMPLETED" and
            self.carrier_odom is not None
        ):
            self.publisher.publish(self._build_attached_odom(self.get_clock().now().to_msg()))
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.world_frame
        odom.child_frame_id = f"{self.uav_name}/base_link"
        odom.pose.pose.position.x = self.world_offset[0] + float(msg.position[0])
        odom.pose.pose.position.y = self.world_offset[1] + float(msg.position[1])
        odom.pose.pose.position.z = self.world_offset[2] - float(msg.position[2])
        odom.twist.twist.linear.x = float(msg.velocity[0])
        odom.twist.twist.linear.y = float(msg.velocity[1])
        odom.twist.twist.linear.z = -float(msg.velocity[2])
        odom.pose.pose.orientation.w = float(msg.q[0])
        odom.pose.pose.orientation.x = float(msg.q[1])
        odom.pose.pose.orientation.y = float(msg.q[2])
        odom.pose.pose.orientation.z = float(msg.q[3])
        odom.twist.twist.angular.x = float(msg.angular_velocity[0])
        odom.twist.twist.angular.y = float(msg.angular_velocity[1])
        odom.twist.twist.angular.z = float(msg.angular_velocity[2])

        should_freeze = (
            not self.controller_active and
            self.arming_state == VehicleStatus.ARMING_STATE_DISARMED
        )
        if should_freeze:
            if self.frozen_odom is None:
                self.frozen_odom = odom
            frozen = Odometry()
            frozen.header.stamp = odom.header.stamp
            frozen.header.frame_id = odom.header.frame_id
            frozen.child_frame_id = odom.child_frame_id
            frozen.pose = self.frozen_odom.pose
            frozen.twist.twist.linear.x = 0.0
            frozen.twist.twist.linear.y = 0.0
            frozen.twist.twist.linear.z = 0.0
            frozen.twist.twist.angular.x = 0.0
            frozen.twist.twist.angular.y = 0.0
            frozen.twist.twist.angular.z = 0.0
            self.publisher.publish(frozen)
            return

        self.frozen_odom = odom
        self.publisher.publish(odom)


def main() -> None:
    rclpy.init()
    node = Px4OdomBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
