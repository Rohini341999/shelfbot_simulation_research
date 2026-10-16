"""
shelf_detector.launch.py — Start only the shelf_detector node.

Run this in a separate terminal AFTER gazebo.launch.py is fully up
(i.e. after the bridge is running and /camera/depth_image is publishing):

    # Terminal 1
    ros2 launch shelfbot_sim gazebo.launch.py

    # Terminal 2 (wait ~10 s for Gazebo + bridge to stabilise)
    ros2 launch shelfbot_sim shelf_detector.launch.py

Check it is working:
    ros2 topic echo /shelf_detector/status
    ros2 topic echo /shelf_detector/rail_pose
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock",
    )

    shelf_detector_node = Node(
        package="shelfbot_sim",
        executable="shelf_detector.py",
        name="shelf_detector",
        output="screen",
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        shelf_detector_node,
    ])
