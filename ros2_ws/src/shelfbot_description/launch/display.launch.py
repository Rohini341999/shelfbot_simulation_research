"""
display.launch.py — Bring up ShelfBot in RViz2 with joint sliders

Nodes launched:
  1. robot_state_publisher  — reads URDF, publishes /tf and /robot_description
  2. joint_state_publisher_gui — provides an interactive slider window to pose joints
  3. rviz2                  — 3D visualiser with pre-configured ShelfBot view

Usage (after building and sourcing the workspace):
    ros2 launch shelfbot_description display.launch.py

Optional arg:
    ros2 launch shelfbot_description display.launch.py use_gui:=false
    (replaces joint_state_publisher_gui with the headless joint_state_publisher)
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare("shelfbot_description")

    # ── Arguments ────────────────────────────────────────────────────────────
    use_gui_arg = DeclareLaunchArgument(
        "use_gui",
        default_value="true",
        description="Launch joint_state_publisher_gui (true) or headless (false)",
    )

    # ── Process xacro → URDF string ──────────────────────────────────────────
    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg_share, "urdf", "shelfbot.urdf.xacro"]),
        ]),
        value_type=str,
    )

    # ── robot_state_publisher ─────────────────────────────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
    )

    # ── joint_state_publisher (always launched — sets all joints to zero) ─────
    # Use the headless version; joint_state_publisher_gui requires a separate install:
    #   sudo apt install ros-jazzy-joint-state-publisher-gui
    # then relaunch with: use_gui:=true
    jsp_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        condition=UnlessCondition(LaunchConfiguration("use_gui")),
    )

    # ── joint_state_publisher_gui (only if requested AND installed) ───────────
    jsp_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(LaunchConfiguration("use_gui")),
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_config = PathJoinSubstitution([pkg_share, "rviz", "shelfbot.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen",
    )

    return LaunchDescription([
        use_gui_arg,
        rsp_node,
        jsp_gui_node,
        jsp_node,
        rviz_node,
    ])
