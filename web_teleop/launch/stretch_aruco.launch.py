import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    dict_file_path = os.path.join(
        get_package_share_directory("web_teleop"), "config", "stretch_marker_dict.yaml"
    )

    detect_aruco_markers = Node(
        package="stretch_core",
        executable="detect_aruco_markers",
        output="screen",
        parameters=[dict_file_path],
    )

    return LaunchDescription(
        [
            detect_aruco_markers,
        ]
    )
