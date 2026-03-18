#!/usr/bin/env python3

import copy
from collections import deque

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray


class DockingVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("docking_visualizer")
        self.declare_parameter("world_frame", "map")
        self.declare_parameter("desired_relative_position", [0.0, 0.0, 0.6])
        self.world_frame = self.get_parameter("world_frame").get_parameter_value().string_value
        self.desired_relative_position = list(
            self.get_parameter("desired_relative_position").get_parameter_value().double_array_value
        )
        self.path_points = deque(maxlen=200)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self._publish_static_world_frame()

        self.create_subscription(Odometry, "/carrier/odom", self._carrier_cb, 10)
        self.create_subscription(Odometry, "/mini/odom", self._mini_cb, 10)
        self.create_subscription(PoseStamped, "/mini/setpoint/pose", self._mini_setpoint_cb, 10)
        self.create_subscription(PoseStamped, "/mini/glide_target", self._mini_glide_target_cb, 10)
        self.create_subscription(Path, "/mini/glide_path", self._mini_glide_path_cb, 10)

        self.carrier_pose_pub = self.create_publisher(PoseStamped, "/carrier/pose", 10)
        self.mini_pose_pub = self.create_publisher(PoseStamped, "/mini/pose", 10)
        self.mini_path_pub = self.create_publisher(Path, "/mini/trajectory", 10)
        self.docking_zone_pub = self.create_publisher(Marker, "/docking_zone_marker", 10)
        self.relative_marker_pub = self.create_publisher(MarkerArray, "/relative_distance_marker", 10)
        self.uav_marker_pub = self.create_publisher(MarkerArray, "/uav_markers", 10)
        self.glide_path_pub = self.create_publisher(Path, "/mini/glide_path_viz", 10)

        self.carrier_pose = None
        self.mini_pose = None
        self.mini_setpoint = None
        self.mini_glide_target = None
        self.mini_glide_path = None

    def _publish_static_world_frame(self) -> None:
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.world_frame
        transform.child_frame_id = "world"
        transform.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform(transform)

    def _carrier_cb(self, msg: Odometry) -> None:
        self.carrier_pose = msg.pose.pose
        self.carrier_pose_pub.publish(self._pose_stamped(msg))
        self._publish_tf("carrier/base_link", msg)
        self._publish_markers()

    def _mini_cb(self, msg: Odometry) -> None:
        self.mini_pose = msg.pose.pose
        pose_msg = self._pose_stamped(msg)
        self.mini_pose_pub.publish(pose_msg)
        self.path_points.append(pose_msg)
        self._publish_path()
        self._publish_tf("mini/base_link", msg)
        self._publish_markers()

    def _mini_setpoint_cb(self, msg: PoseStamped) -> None:
        self.mini_setpoint = msg
        self._publish_markers()

    def _mini_glide_target_cb(self, msg: PoseStamped) -> None:
        self.mini_glide_target = msg
        self._publish_markers()

    def _mini_glide_path_cb(self, msg: Path) -> None:
        self.mini_glide_path = msg
        self.glide_path_pub.publish(msg)

    def _pose_stamped(self, msg: Odometry) -> PoseStamped:
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        if not pose.header.frame_id:
          pose.header.frame_id = self.world_frame
        return pose

    def _publish_tf(self, child_frame: str, msg: Odometry) -> None:
        transform = TransformStamped()
        transform.header = msg.header
        transform.header.frame_id = transform.header.frame_id or self.world_frame
        transform.child_frame_id = child_frame
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self.tf_broadcaster.sendTransform(transform)

    def _publish_path(self) -> None:
        path = Path()
        path.header.frame_id = self.world_frame
        path.header.stamp = self.get_clock().now().to_msg()
        path.poses = list(self.path_points)
        self.mini_path_pub.publish(path)

    def _publish_markers(self) -> None:
        if self.carrier_pose is None:
            return

        docking_zone = Marker()
        docking_zone.header.frame_id = self.world_frame
        docking_zone.header.stamp = self.get_clock().now().to_msg()
        docking_zone.ns = "docking_zone"
        docking_zone.id = 0
        docking_zone.type = Marker.CYLINDER
        docking_zone.action = Marker.ADD
        docking_zone.pose.position.x = self.carrier_pose.position.x + self.desired_relative_position[0]
        docking_zone.pose.position.y = self.carrier_pose.position.y + self.desired_relative_position[1]
        docking_zone.pose.position.z = self.carrier_pose.position.z + self.desired_relative_position[2]
        docking_zone.pose.orientation.w = 1.0
        docking_zone.scale.x = 0.6
        docking_zone.scale.y = 0.6
        docking_zone.scale.z = 0.15
        docking_zone.color.a = 0.35
        docking_zone.color.r = 0.0
        docking_zone.color.g = 0.8
        docking_zone.color.b = 0.2
        self.docking_zone_pub.publish(docking_zone)

        markers = MarkerArray()
        if self.mini_pose is not None:
            line = Marker()
            line.header.frame_id = self.world_frame
            line.header.stamp = docking_zone.header.stamp
            line.ns = "relative_distance"
            line.id = 0
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.05
            line.color.a = 1.0
            line.color.r = 1.0
            line.color.g = 0.8
            line.color.b = 0.0
            line.points = [self.carrier_pose.position, self.mini_pose.position]
            markers.markers.append(line)

        target_pose = self.mini_glide_target if self.mini_glide_target is not None else self.mini_setpoint
        if target_pose is not None:
            target = Marker()
            target.header = target_pose.header
            target.ns = "mini_setpoint"
            target.id = 1
            target.type = Marker.SPHERE
            target.action = Marker.ADD
            target.pose = target_pose.pose
            target.scale.x = 0.18
            target.scale.y = 0.18
            target.scale.z = 0.18
            target.color.a = 0.9
            target.color.r = 0.1
            target.color.g = 0.4
            target.color.b = 1.0
            markers.markers.append(target)

        self.relative_marker_pub.publish(markers)
        self._publish_uav_markers(docking_zone.header.stamp)

    def _publish_uav_markers(self, stamp) -> None:
        markers = MarkerArray()

        if self.carrier_pose is not None:
            markers.markers.extend(self._vehicle_markers(
                stamp=stamp,
                marker_id_base=0,
                ns="carrier",
                pose=self.carrier_pose,
                color=(0.1, 0.9, 0.2),
                label="Carrier",
            ))

        if self.mini_pose is not None:
            markers.markers.extend(self._vehicle_markers(
                stamp=stamp,
                marker_id_base=10,
                ns="mini",
                pose=self.mini_pose,
                color=(1.0, 0.2, 0.2),
                label="Mini",
            ))

        self.uav_marker_pub.publish(markers)

    def _vehicle_markers(self, stamp, marker_id_base: int, ns: str, pose, color, label: str):
        body = Marker()
        body.header.frame_id = self.world_frame
        body.header.stamp = stamp
        body.ns = ns
        body.id = marker_id_base
        body.type = Marker.ARROW
        body.action = Marker.ADD
        body.pose = copy.deepcopy(pose)
        body.scale.x = 0.8
        body.scale.y = 0.18
        body.scale.z = 0.18
        body.color.a = 1.0
        body.color.r = color[0]
        body.color.g = color[1]
        body.color.b = color[2]

        text = Marker()
        text.header.frame_id = self.world_frame
        text.header.stamp = stamp
        text.ns = ns
        text.id = marker_id_base + 1
        text.type = Marker.TEXT_VIEW_FACING
        text.action = Marker.ADD
        text.pose = copy.deepcopy(pose)
        text.pose.position.z += 0.5
        text.scale.z = 0.3
        text.color.a = 1.0
        text.color.r = 1.0
        text.color.g = 1.0
        text.color.b = 1.0
        text.text = label
        return [body, text]


def main() -> None:
    rclpy.init()
    node = DockingVisualizer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
