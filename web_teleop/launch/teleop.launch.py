import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
    stretch_core_dir = get_package_share_directory("stretch_core")
    rosbridge_server_dir = get_package_share_directory("rosbridge_server")
    web_teleop_dir = get_package_share_directory("web_teleop")

    # stretch_core driver
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stretch_core_dir, "launch", "stretch_driver.launch.py")
        )
    )

    # rosbridge websocket server with SSL
    rosbridge_websocket_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                rosbridge_server_dir, "launch", "rosbridge_websocket_launch.xml"
            )
        ),
    )

    # multi-camera streamer
    multi_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(web_teleop_dir, "launch", "multi_camera.launch.py")
        )
    )

    # optional node to play a sound
    play_sound_node = Node(
        package="web_teleop",
        executable="play_sound",
    )

    return LaunchDescription(
        [
            stretch_driver_launch,
            rosbridge_websocket_launch,
            multi_cam_launch,
            play_sound_node,
        ]
    )
