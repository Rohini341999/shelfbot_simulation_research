"""
gazebo.launch.py — Launch ShelfBot in Gazebo Harmonic (gz-sim8)

Starts: Gazebo world, robot_state_publisher, spawn robot, ros_gz_bridge,
        climb_controller.

shelf_detector is intentionally NOT launched here — start it separately to
avoid crashing the bridge process:
    ros2 launch shelfbot_sim shelf_detector.launch.py

Fixes applied (v4):
  - ParameterValue wrapping for robot_description (Jazzy requirement)
  - GZ_SIM_RESOURCE_PATH set so package:// mesh URIs resolve in Gazebo
  - TimerAction delays: Gazebo 5 s, bridge 6 s, climb_controller 7 s

Drive the robot:
    ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.3}}' --once

Trigger climb sequence (level height in metres):
    ros2 topic pub /climb_controller/start std_msgs/msg/Float32 '{data: 0.55}' --once

Watch state:
    ros2 topic echo /climb_controller/state
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    SetEnvironmentVariable, TimerAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command, EnvironmentVariable, FindExecutable,
    LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    desc_share = FindPackageShare("shelfbot_description")
    sim_share  = FindPackageShare("shelfbot_sim")

    # ── GZ_SIM_RESOURCE_PATH: lets Gazebo resolve package:// mesh URIs ──────
    # Must include the *parent* of the package share dir, i.e. .../share/
    desc_share_dir = get_package_share_directory("shelfbot_description")
    gz_resource_parent = os.path.dirname(desc_share_dir)   # .../install/shelfbot_description/share
    existing_gz_path   = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    new_gz_path        = (
        gz_resource_parent
        + os.pathsep + "/opt/ros/jazzy/share"
        + (os.pathsep + existing_gz_path if existing_gz_path else "")
    )
    set_gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", new_gz_path)

    # ── Arguments ─────────────────────────────────────────────────────────
    use_sim_time_arg  = DeclareLaunchArgument("use_sim_time", default_value="true")
    robot_x_arg   = DeclareLaunchArgument("robot_x",   default_value="0.0")
    robot_y_arg   = DeclareLaunchArgument("robot_y",   default_value="-0.80")
    robot_z_arg   = DeclareLaunchArgument("robot_z",   default_value="0.076")
    robot_yaw_arg = DeclareLaunchArgument("robot_yaw", default_value="1.5708")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # ── URDF via xacro ─────────────────────────────────────────────────────
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([desc_share, "urdf", "shelfbot.urdf.xacro"]),
        ]),
        value_type=str,
    )

    # ── robot_state_publisher ──────────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "use_sim_time": use_sim_time,
        }],
    )

    # ── Gazebo Harmonic ────────────────────────────────────────────────────
    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"), "launch", "gz_sim.launch.py"
            ])
        ]),
        launch_arguments={
            "gz_args": [
                PathJoinSubstitution([sim_share, "worlds", "shelfbot_store_aisle.sdf"]),
                " -r",
            ],
        }.items(),
    )

    # ── Spawn robot (delay 5 s — Gazebo needs time to fully start) ─────────
    spawn_node = TimerAction(
        period=5.0,
        actions=[Node(
            package="ros_gz_sim",
            executable="create",
            name="spawn_shelfbot",
            output="screen",
            arguments=[
                "-name",  "shelfbot",
                "-topic", "/robot_description",
                "-x",     LaunchConfiguration("robot_x"),
                "-y",     LaunchConfiguration("robot_y"),
                "-z",     LaunchConfiguration("robot_z"),
                "-Y",     LaunchConfiguration("robot_yaw"),
            ],
        )],
    )

    # ── ros_gz_bridge ──────────────────────────────────────────────────────
    bridge_config = PathJoinSubstitution([sim_share, "config", "gz_bridge.yaml"])
    bridge_node = TimerAction(
        period=6.0,
        actions=[Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            name="gz_ros_bridge",
            output="screen",
            parameters=[{
                "config_file": bridge_config,
                "use_sim_time": use_sim_time,
            }],
        )],
    )

    # ── climb_controller ───────────────────────────────────────────────────
    # shelf_detector is launched separately:
    #   ros2 launch shelfbot_sim shelf_detector.launch.py
    climb_controller_node = TimerAction(
        period=7.0,
        actions=[Node(
            package="shelfbot_sim",
            executable="climb_controller.py",
            name="climb_controller",
            output="screen",
            parameters=[{"use_sim_time": use_sim_time}],
        )],
    )

    return LaunchDescription([
        set_gz_resource,          # must be first — sets env before Gazebo starts
        use_sim_time_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        robot_yaw_arg,
        rsp_node,
        gz_launch,
        spawn_node,
        bridge_node,
        climb_controller_node,
    ])
