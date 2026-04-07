from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_agent = LaunchConfiguration("start_agent")
    start_px4_bridge = LaunchConfiguration("start_px4_bridge")
    use_mock_sim = LaunchConfiguration("use_mock_sim")
    use_px4_odom_bridge = LaunchConfiguration("use_px4_odom_bridge")
    start_rviz = LaunchConfiguration("start_rviz")
    carrier_activate_on_launch = LaunchConfiguration("carrier_activate_on_launch")
    carrier_use_position_setpoint = LaunchConfiguration("carrier_use_position_setpoint")
    mini_use_offboard_orbit_hold = LaunchConfiguration("mini_use_offboard_orbit_hold")
    mini_orbit_hold_ready_altitude = LaunchConfiguration("mini_orbit_hold_ready_altitude")
    mini_orbit_hold_enable_delay_sec = LaunchConfiguration("mini_orbit_hold_enable_delay_sec")
    mini_allow_orbit_recentering = LaunchConfiguration("mini_allow_orbit_recentering")
    carrier_offset_auto = LaunchConfiguration("carrier_offset_auto")
    carrier_outside_margin = LaunchConfiguration("carrier_outside_margin")
    carrier_outside_angle_deg = LaunchConfiguration("carrier_outside_angle_deg")
    carrier_approach_speed_limit = LaunchConfiguration("carrier_approach_speed_limit")
    carrier_tracking_speed_limit = LaunchConfiguration("carrier_tracking_speed_limit")
    carrier_docking_speed_limit = LaunchConfiguration("carrier_docking_speed_limit")
    carrier_offset_x = LaunchConfiguration("carrier_offset_x")
    carrier_offset_y = LaunchConfiguration("carrier_offset_y")
    carrier_offset_z = LaunchConfiguration("carrier_offset_z")
    mini_takeoff_altitude = LaunchConfiguration("mini_takeoff_altitude")
    mini_orbit_center_x = LaunchConfiguration("mini_orbit_center_x")
    mini_orbit_center_y = LaunchConfiguration("mini_orbit_center_y")
    mini_orbit_start_phase_deg = LaunchConfiguration("mini_orbit_start_phase_deg")
    mini_orbit_radius = LaunchConfiguration("mini_orbit_radius")
    mini_orbit_speed = LaunchConfiguration("mini_orbit_speed")
    mini_loiter_speed_command = LaunchConfiguration("mini_loiter_speed_command")
    mini_glide_speed_command = LaunchConfiguration("mini_glide_speed_command")
    mini_glide_trigger_phase = LaunchConfiguration("mini_glide_trigger_phase")
    mini_glide_release_enabled = LaunchConfiguration("mini_glide_release_enabled")
    mini_glide_release_mode = LaunchConfiguration("mini_glide_release_mode")
    mini_tracking_speed_command = LaunchConfiguration("mini_tracking_speed_command")
    mini_docking_speed_command = LaunchConfiguration("mini_docking_speed_command")
    mini_capture_speed_command = LaunchConfiguration("mini_capture_speed_command")
    mini_glide_trigger_distance = LaunchConfiguration("mini_glide_trigger_distance")
    mini_capture_distance = LaunchConfiguration("mini_capture_distance")
    mini_terminal_slowdown_start_distance = LaunchConfiguration("mini_terminal_slowdown_start_distance")
    mini_terminal_slowdown_finish_distance = LaunchConfiguration("mini_terminal_slowdown_finish_distance")
    mini_terminal_slowdown_max_abs_y = LaunchConfiguration("mini_terminal_slowdown_max_abs_y")
    mini_glide_tangent_exit_sync_gate_distance_cap = LaunchConfiguration("mini_glide_tangent_exit_sync_gate_distance_cap")
    mini_glide_tangent_exit_min_hold_sec = LaunchConfiguration("mini_glide_tangent_exit_min_hold_sec")
    mini_glide_tangent_exit_release_distance_max = LaunchConfiguration("mini_glide_tangent_exit_release_distance_max")
    mini_glide_tangent_exit_release_height_error_max = LaunchConfiguration("mini_glide_tangent_exit_release_height_error_max")
    mini_glide_tangent_exit_release_progress_min = LaunchConfiguration("mini_glide_tangent_exit_release_progress_min")
    mini_tracking_speed = LaunchConfiguration("mini_tracking_speed")
    mini_docking_speed = LaunchConfiguration("mini_docking_speed")
    mini_capture_speed = LaunchConfiguration("mini_capture_speed")
    mini_slowdown_start_distance = LaunchConfiguration("mini_slowdown_start_distance")
    mini_slowdown_finish_distance = LaunchConfiguration("mini_slowdown_finish_distance")
    mini_max_accel = LaunchConfiguration("mini_max_accel")
    carrier_max_accel = LaunchConfiguration("carrier_max_accel")
    carrier_max_speed_xy = LaunchConfiguration("carrier_max_speed_xy")
    carrier_max_speed_z = LaunchConfiguration("carrier_max_speed_z")
    attach_distance = LaunchConfiguration("attach_distance")
    attach_speed_threshold = LaunchConfiguration("attach_speed_threshold")
    soft_attach_distance = LaunchConfiguration("soft_attach_distance")
    soft_attach_xy_tolerance = LaunchConfiguration("soft_attach_xy_tolerance")
    soft_attach_z_min = LaunchConfiguration("soft_attach_z_min")
    soft_attach_z_max = LaunchConfiguration("soft_attach_z_max")
    soft_attach_vx_threshold = LaunchConfiguration("soft_attach_vx_threshold")
    soft_attach_vy_threshold = LaunchConfiguration("soft_attach_vy_threshold")
    soft_attach_vz_threshold = LaunchConfiguration("soft_attach_vz_threshold")

    docking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("easydocking_control"),
                "launch",
                "docking.launch.py",
            ])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_rviz": start_rviz,
            "start_px4_bridge": start_px4_bridge,
            "use_mock_sim": use_mock_sim,
            "use_px4_odom_bridge": use_px4_odom_bridge,
            "carrier_activate_on_launch": carrier_activate_on_launch,
            "carrier_use_position_setpoint": carrier_use_position_setpoint,
            "mini_use_offboard_orbit_hold": mini_use_offboard_orbit_hold,
            "mini_orbit_hold_ready_altitude": mini_orbit_hold_ready_altitude,
            "mini_orbit_hold_enable_delay_sec": mini_orbit_hold_enable_delay_sec,
            "mini_allow_orbit_recentering": mini_allow_orbit_recentering,
            "carrier_offset_auto": carrier_offset_auto,
            "carrier_outside_margin": carrier_outside_margin,
            "carrier_outside_angle_deg": carrier_outside_angle_deg,
            "carrier_approach_speed_limit": carrier_approach_speed_limit,
            "carrier_tracking_speed_limit": carrier_tracking_speed_limit,
            "carrier_docking_speed_limit": carrier_docking_speed_limit,
            "carrier_offset_x": carrier_offset_x,
            "carrier_offset_y": carrier_offset_y,
            "carrier_offset_z": carrier_offset_z,
            "mini_takeoff_altitude": mini_takeoff_altitude,
            "mini_orbit_center_x": mini_orbit_center_x,
            "mini_orbit_center_y": mini_orbit_center_y,
            "mini_orbit_start_phase_deg": mini_orbit_start_phase_deg,
            "mini_orbit_radius": mini_orbit_radius,
            "mini_orbit_speed": mini_orbit_speed,
            "mini_loiter_speed_command": mini_loiter_speed_command,
            "mini_glide_speed_command": mini_glide_speed_command,
            "mini_glide_trigger_phase": mini_glide_trigger_phase,
            "mini_glide_release_enabled": mini_glide_release_enabled,
            "mini_glide_release_mode": mini_glide_release_mode,
            "mini_tracking_speed_command": mini_tracking_speed_command,
            "mini_docking_speed_command": mini_docking_speed_command,
            "mini_capture_speed_command": mini_capture_speed_command,
            "mini_glide_trigger_distance": mini_glide_trigger_distance,
            "mini_capture_distance": mini_capture_distance,
            "mini_terminal_slowdown_start_distance": mini_terminal_slowdown_start_distance,
            "mini_terminal_slowdown_finish_distance": mini_terminal_slowdown_finish_distance,
            "mini_terminal_slowdown_max_abs_y": mini_terminal_slowdown_max_abs_y,
            "mini_glide_tangent_exit_sync_gate_distance_cap": mini_glide_tangent_exit_sync_gate_distance_cap,
            "mini_glide_tangent_exit_min_hold_sec": mini_glide_tangent_exit_min_hold_sec,
            "mini_glide_tangent_exit_release_distance_max": mini_glide_tangent_exit_release_distance_max,
            "mini_glide_tangent_exit_release_height_error_max": mini_glide_tangent_exit_release_height_error_max,
            "mini_glide_tangent_exit_release_progress_min": mini_glide_tangent_exit_release_progress_min,
            "mini_tracking_speed": mini_tracking_speed,
            "mini_docking_speed": mini_docking_speed,
            "mini_capture_speed": mini_capture_speed,
            "mini_slowdown_start_distance": mini_slowdown_start_distance,
            "mini_slowdown_finish_distance": mini_slowdown_finish_distance,
            "mini_max_accel": mini_max_accel,
            "carrier_max_accel": carrier_max_accel,
            "carrier_max_speed_xy": carrier_max_speed_xy,
            "carrier_max_speed_z": carrier_max_speed_z,
            "attach_distance": attach_distance,
            "attach_speed_threshold": attach_speed_threshold,
            "soft_attach_distance": soft_attach_distance,
            "soft_attach_xy_tolerance": soft_attach_xy_tolerance,
            "soft_attach_z_min": soft_attach_z_min,
            "soft_attach_z_max": soft_attach_z_max,
            "soft_attach_vx_threshold": soft_attach_vx_threshold,
            "soft_attach_vy_threshold": soft_attach_vy_threshold,
            "soft_attach_vz_threshold": soft_attach_vz_threshold,
        }.items(),
    )

    microxrce_agent = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"],
        output="screen",
        condition=IfCondition(start_agent),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("start_agent", default_value="false"),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_px4_bridge", default_value="false"),
        DeclareLaunchArgument("use_mock_sim", default_value="true"),
        DeclareLaunchArgument("use_px4_odom_bridge", default_value="false"),
        DeclareLaunchArgument("carrier_activate_on_launch", default_value="true"),
        DeclareLaunchArgument("carrier_use_position_setpoint", default_value="true"),
        DeclareLaunchArgument("mini_use_offboard_orbit_hold", default_value="false"),
        DeclareLaunchArgument("mini_orbit_hold_ready_altitude", default_value="1.5"),
        DeclareLaunchArgument("mini_orbit_hold_enable_delay_sec", default_value="3.0"),
        DeclareLaunchArgument("mini_allow_orbit_recentering", default_value="false"),
        DeclareLaunchArgument("carrier_offset_auto", default_value="true"),
        DeclareLaunchArgument("carrier_outside_margin", default_value="10.0"),
        DeclareLaunchArgument("carrier_outside_angle_deg", default_value="-135.0"),
        DeclareLaunchArgument("carrier_approach_speed_limit", default_value="11.6"),
        DeclareLaunchArgument("carrier_tracking_speed_limit", default_value="10.8"),
        DeclareLaunchArgument("carrier_docking_speed_limit", default_value="9.4"),
        DeclareLaunchArgument("carrier_offset_x", default_value="0.0"),
        DeclareLaunchArgument("carrier_offset_y", default_value="0.0"),
        DeclareLaunchArgument("carrier_offset_z", default_value="0.0"),
        DeclareLaunchArgument("mini_takeoff_altitude", default_value="30.0"),
        DeclareLaunchArgument("mini_orbit_center_x", default_value="10.0"),
        DeclareLaunchArgument("mini_orbit_center_y", default_value="-6.0"),
        DeclareLaunchArgument("mini_orbit_start_phase_deg", default_value="180.0"),
        DeclareLaunchArgument("mini_orbit_radius", default_value="55.0"),
        DeclareLaunchArgument("mini_orbit_speed", default_value="10.0"),
        DeclareLaunchArgument("mini_loiter_speed_command", default_value="12.0"),
        DeclareLaunchArgument("mini_glide_speed_command", default_value="10.0"),
        DeclareLaunchArgument("mini_glide_trigger_phase", default_value="DOCKING"),
        DeclareLaunchArgument("mini_glide_release_enabled", default_value="true"),
        DeclareLaunchArgument("mini_glide_release_mode", default_value="score_state_machine"),
        DeclareLaunchArgument("mini_tracking_speed_command", default_value="10.0"),
        DeclareLaunchArgument("mini_docking_speed_command", default_value="8.0"),
        DeclareLaunchArgument("mini_capture_speed_command", default_value="6.0"),
        DeclareLaunchArgument("mini_glide_trigger_distance", default_value="10.0"),
        DeclareLaunchArgument("mini_capture_distance", default_value="1.6"),
        DeclareLaunchArgument("mini_terminal_slowdown_start_distance", default_value="4.2"),
        DeclareLaunchArgument("mini_terminal_slowdown_finish_distance", default_value="0.8"),
        DeclareLaunchArgument("mini_terminal_slowdown_max_abs_y", default_value="1.0"),
        DeclareLaunchArgument("mini_glide_tangent_exit_sync_gate_distance_cap", default_value="0.0"),
        DeclareLaunchArgument("mini_glide_tangent_exit_min_hold_sec", default_value="5.0"),
        DeclareLaunchArgument("mini_glide_tangent_exit_release_distance_max", default_value="0.0"),
        DeclareLaunchArgument("mini_glide_tangent_exit_release_height_error_max", default_value="0.0"),
        DeclareLaunchArgument("mini_glide_tangent_exit_release_progress_min", default_value="-1.0"),
        DeclareLaunchArgument("mini_tracking_speed", default_value="9.7"),
        DeclareLaunchArgument("mini_docking_speed", default_value="6.6"),
        DeclareLaunchArgument("mini_capture_speed", default_value="4.8"),
        DeclareLaunchArgument("mini_slowdown_start_distance", default_value="5.0"),
        DeclareLaunchArgument("mini_slowdown_finish_distance", default_value="2.0"),
        DeclareLaunchArgument("mini_max_accel", default_value="2.0"),
        DeclareLaunchArgument("carrier_max_accel", default_value="2.5"),
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
        microxrce_agent,
        docking_launch,
    ])
