import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_float(context, name: str) -> float:
    return float(LaunchConfiguration(name).perform(context))


def _launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rviz = LaunchConfiguration("start_rviz")
    start_px4_bridge = LaunchConfiguration("start_px4_bridge")
    use_mock_sim = LaunchConfiguration("use_mock_sim")
    use_px4_odom_bridge = LaunchConfiguration("use_px4_odom_bridge")
    carrier_activate_on_launch = LaunchConfiguration("carrier_activate_on_launch")
    mini_use_offboard_orbit_hold = LaunchConfiguration("mini_use_offboard_orbit_hold")
    mini_orbit_hold_ready_altitude = LaunchConfiguration("mini_orbit_hold_ready_altitude")
    mini_orbit_hold_enable_delay_sec = LaunchConfiguration("mini_orbit_hold_enable_delay_sec")
    mini_allow_orbit_recentering = LaunchConfiguration("mini_allow_orbit_recentering")
    carrier_approach_speed_limit = _as_float(context, "carrier_approach_speed_limit")
    carrier_tracking_speed_limit = _as_float(context, "carrier_tracking_speed_limit")
    carrier_docking_speed_limit = _as_float(context, "carrier_docking_speed_limit")
    mini_takeoff_altitude = _as_float(context, "mini_takeoff_altitude")
    mini_orbit_radius = _as_float(context, "mini_orbit_radius")
    mini_orbit_speed = _as_float(context, "mini_orbit_speed")
    mini_loiter_speed_command = _as_float(context, "mini_loiter_speed_command")
    mini_tracking_speed = _as_float(context, "mini_tracking_speed")
    mini_docking_speed = _as_float(context, "mini_docking_speed")
    mini_capture_speed = _as_float(context, "mini_capture_speed")
    mini_slowdown_start_distance = _as_float(context, "mini_slowdown_start_distance")
    mini_slowdown_finish_distance = _as_float(context, "mini_slowdown_finish_distance")
    mini_max_accel = _as_float(context, "mini_max_accel")
    carrier_max_accel = _as_float(context, "carrier_max_accel")
    carrier_max_speed_xy = _as_float(context, "carrier_max_speed_xy")
    carrier_max_speed_z = _as_float(context, "carrier_max_speed_z")
    attach_distance = _as_float(context, "attach_distance")
    attach_speed_threshold = _as_float(context, "attach_speed_threshold")
    soft_attach_distance = _as_float(context, "soft_attach_distance")
    soft_attach_xy_tolerance = _as_float(context, "soft_attach_xy_tolerance")
    soft_attach_z_min = _as_float(context, "soft_attach_z_min")
    soft_attach_z_max = _as_float(context, "soft_attach_z_max")
    soft_attach_vx_threshold = _as_float(context, "soft_attach_vx_threshold")
    soft_attach_vy_threshold = _as_float(context, "soft_attach_vy_threshold")
    soft_attach_vz_threshold = _as_float(context, "soft_attach_vz_threshold")

    carrier_world_offset = [
        _as_float(context, "carrier_offset_x"),
        _as_float(context, "carrier_offset_y"),
        _as_float(context, "carrier_offset_z"),
    ]
    mini_orbit_start_phase_deg = _as_float(context, "mini_orbit_start_phase_deg")
    mini_orbit_center = [
        _as_float(context, "mini_orbit_center_x"),
        _as_float(context, "mini_orbit_center_y"),
    ]

    package_share = get_package_share_directory("easydocking_control")
    rviz_config = os.path.join(package_share, "config", "docking.rviz")

    controller = Node(
        package="easydocking_control",
        executable="docking_controller_node",
        name="docking_controller",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "approach_distance": 24.0,
            "tracking_distance": 10.0,
            "docking_distance": 1.2,
            "idle_hover_altitude": 0.4,
            "control_rate": 50.0,
            "k1": 1.0,
            "k2": 2.0,
            "adaptive_gain": 0.5,
            "guidance_gain": 1.0,
            "world_frame": "map",
            "passive_target_mode": True,
            "desired_relative_position": [0.0, 0.0, 0.4],
            "terminal_relative_position": [0.0, 0.0, 0.2],
            "carrier_approach_speed_limit": carrier_approach_speed_limit,
            "carrier_tracking_speed_limit": carrier_tracking_speed_limit,
            "carrier_docking_speed_limit": carrier_docking_speed_limit,
            "intercept_lookahead": 1.6,
            "docking_speed_threshold": 1.0,
        }],
    )

    visualizer = Node(
        package="easydocking_control",
        executable="rviz_visualizer.py",
        name="docking_visualizer",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "world_frame": "map",
            "desired_relative_position": [0.0, 0.0, 0.2],
        }],
    )

    px4_bridge_mini = Node(
        package="easydocking_control",
        executable="px4_fixed_wing_bridge.py",
        name="mini_fixed_wing_bridge",
        output="screen",
        condition=IfCondition(start_px4_bridge),
        parameters=[{
            "use_sim_time": use_sim_time,
            "uav_name": "mini",
            "px4_namespace": "/px4_2",
            "vehicle_id": 3,
            "world_offset": [0.0, 0.0, 1.2],
            "takeoff_altitude": mini_takeoff_altitude,
            "orbit_center": mini_orbit_center,
            "orbit_radius": mini_orbit_radius,
            "orbit_speed": mini_orbit_speed,
            "glide_entry_distance": 20.0,
            "glide_entry_height": 4.0,
            "final_relative_position": [0.0, 0.0, 0.2],
            "glide_speed": 10.0,
            "glide_descent_rate": 0.4,
            "glide_trigger_distance": 4.5,
            "glide_trigger_phase": "DOCKING",
            "auto_takeoff_on_launch": True,
            "orbit_start_phase_deg": mini_orbit_start_phase_deg,
            "loiter_speed_command": mini_loiter_speed_command,
            "glide_speed_command": 10.0,
            "tracking_speed_command": 9.6,
            "docking_speed_command": 8.8,
            "capture_speed_command": 8.0,
            "capture_distance": 2.0,
            "terminal_slowdown_start_distance": 3.0,
            "terminal_slowdown_finish_distance": 0.9,
            "terminal_slowdown_max_abs_y": 1.0,
            "use_offboard_orbit_hold": mini_use_offboard_orbit_hold,
            "orbit_hold_ready_altitude": mini_orbit_hold_ready_altitude,
            "orbit_hold_enable_delay_sec": mini_orbit_hold_enable_delay_sec,
            "orbit_radial_gain": 0.32,
            "orbit_radial_speed_limit": 1.6,
            "orbit_phase_lead_deg": 16.0,
            "allow_orbit_recentering": mini_allow_orbit_recentering,
        }],
    )

    px4_bridge_carrier = Node(
        package="easydocking_control",
        executable="px4_offboard_bridge.py",
        name="carrier_px4_bridge",
        output="screen",
        condition=IfCondition(start_px4_bridge),
        parameters=[{
            "use_sim_time": use_sim_time,
            "uav_name": "carrier",
            "px4_namespace": "/px4_1",
            "vehicle_id": 2,
            "arm_on_start": True,
            "world_offset": carrier_world_offset,
            "use_velocity_feedforward": True,
            "activate_on_launch": carrier_activate_on_launch,
        }],
    )

    mock_sim = Node(
        package="easydocking_control",
        executable="simple_dual_uav_sim.py",
        name="simple_dual_uav_sim",
        output="screen",
        condition=IfCondition(use_mock_sim),
        parameters=[{
            "use_sim_time": use_sim_time,
            "world_frame": "map",
            "carrier_initial": [
                carrier_world_offset[0],
                carrier_world_offset[1],
                0.4 + carrier_world_offset[2],
            ],
            "mini_initial": [
                mini_orbit_center[0] + mini_orbit_radius,
                mini_orbit_center[1],
                0.0,
            ],
            "mini_mode": "fixed_wing_orbit",
            "mini_takeoff_altitude": mini_takeoff_altitude,
            "mini_orbit_center": [mini_orbit_center[0], mini_orbit_center[1], mini_takeoff_altitude],
            "mini_orbit_radius": mini_orbit_radius,
            "mini_orbit_speed": mini_orbit_speed,
            "mini_tracking_speed": mini_tracking_speed,
            "mini_docking_speed": mini_docking_speed,
            "mini_capture_speed": mini_capture_speed,
            "mini_slowdown_start_distance": mini_slowdown_start_distance,
            "mini_slowdown_finish_distance": mini_slowdown_finish_distance,
            "terminal_straight_trigger_distance": 1.35,
            "mini_max_accel": mini_max_accel,
            "carrier_max_accel": carrier_max_accel,
            "carrier_max_speed_xy": carrier_max_speed_xy,
            "carrier_max_speed_z": carrier_max_speed_z,
            "attach_relative_position": [0.0, 0.0, 0.2],
            "attach_distance": attach_distance,
            "attach_speed_threshold": attach_speed_threshold,
            "soft_attach_distance": soft_attach_distance,
            "soft_attach_xy_tolerance": soft_attach_xy_tolerance,
            "soft_attach_z_min": soft_attach_z_min,
            "soft_attach_z_max": soft_attach_z_max,
            "soft_attach_vx_threshold": soft_attach_vx_threshold,
            "soft_attach_vy_threshold": soft_attach_vy_threshold,
            "soft_attach_vz_threshold": soft_attach_vz_threshold,
        }],
    )

    carrier_odom_bridge = Node(
        package="easydocking_control",
        executable="px4_odom_bridge.py",
        name="carrier_odom_bridge",
        output="screen",
        condition=IfCondition(use_px4_odom_bridge),
        parameters=[{
            "use_sim_time": use_sim_time,
            "uav_name": "carrier",
            "px4_namespace": "/px4_1",
            "world_frame": "map",
            "world_offset": carrier_world_offset,
        }],
    )

    mini_odom_bridge = Node(
        package="easydocking_control",
        executable="px4_odom_bridge.py",
        name="mini_odom_bridge",
        output="screen",
        condition=IfCondition(use_px4_odom_bridge),
        parameters=[{
            "use_sim_time": use_sim_time,
            "uav_name": "mini",
            "px4_namespace": "/px4_2",
            "world_frame": "map",
            "world_offset": [0.0, 0.0, 1.2],
            "attach_relative_position": [0.0, 0.0, 0.2],
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
        condition=IfCondition(start_rviz),
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return [
        controller,
        visualizer,
        mock_sim,
        px4_bridge_mini,
        px4_bridge_carrier,
        carrier_odom_bridge,
        mini_odom_bridge,
        rviz,
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_px4_bridge", default_value="false"),
        DeclareLaunchArgument("use_mock_sim", default_value="true"),
        DeclareLaunchArgument("use_px4_odom_bridge", default_value="false"),
        DeclareLaunchArgument("carrier_activate_on_launch", default_value="false"),
        DeclareLaunchArgument("mini_use_offboard_orbit_hold", default_value="false"),
        DeclareLaunchArgument("mini_orbit_hold_ready_altitude", default_value="29.0"),
        DeclareLaunchArgument("mini_orbit_hold_enable_delay_sec", default_value="4.0"),
        DeclareLaunchArgument("mini_allow_orbit_recentering", default_value="false"),
        DeclareLaunchArgument("carrier_approach_speed_limit", default_value="11.6"),
        DeclareLaunchArgument("carrier_tracking_speed_limit", default_value="10.8"),
        DeclareLaunchArgument("carrier_docking_speed_limit", default_value="9.4"),
        DeclareLaunchArgument("mini_takeoff_altitude", default_value="30.0"),
        DeclareLaunchArgument("mini_orbit_radius", default_value="55.0"),
        DeclareLaunchArgument("mini_orbit_speed", default_value="10.0"),
        DeclareLaunchArgument("mini_loiter_speed_command", default_value="12.0"),
        DeclareLaunchArgument("mini_tracking_speed", default_value="9.7"),
        DeclareLaunchArgument("mini_docking_speed", default_value="6.6"),
        DeclareLaunchArgument("mini_capture_speed", default_value="4.8"),
        DeclareLaunchArgument("mini_slowdown_start_distance", default_value="5.0"),
        DeclareLaunchArgument("mini_slowdown_finish_distance", default_value="2.0"),
        DeclareLaunchArgument("mini_max_accel", default_value="2.0"),
        DeclareLaunchArgument("carrier_max_accel", default_value="3.2"),
        DeclareLaunchArgument("carrier_max_speed_xy", default_value="11.8"),
        DeclareLaunchArgument("carrier_max_speed_z", default_value="2.2"),
        DeclareLaunchArgument("attach_distance", default_value="0.24"),
        DeclareLaunchArgument("attach_speed_threshold", default_value="0.8"),
        DeclareLaunchArgument("soft_attach_distance", default_value="0.34"),
        DeclareLaunchArgument("soft_attach_xy_tolerance", default_value="0.16"),
        DeclareLaunchArgument("soft_attach_z_min", default_value="0.12"),
        DeclareLaunchArgument("soft_attach_z_max", default_value="0.32"),
        DeclareLaunchArgument("soft_attach_vx_threshold", default_value="0.9"),
        DeclareLaunchArgument("soft_attach_vy_threshold", default_value="0.9"),
        DeclareLaunchArgument("soft_attach_vz_threshold", default_value="0.35"),
        DeclareLaunchArgument("carrier_offset_x", default_value="54.0"),
        DeclareLaunchArgument("carrier_offset_y", default_value="-30.0"),
        DeclareLaunchArgument("carrier_offset_z", default_value="0.0"),
        DeclareLaunchArgument("mini_orbit_center_x", default_value="10.0"),
        DeclareLaunchArgument("mini_orbit_center_y", default_value="-6.0"),
        DeclareLaunchArgument("mini_orbit_start_phase_deg", default_value="180.0"),
        OpaqueFunction(function=_launch_setup),
    ])
