from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stretch_core_dir = get_package_share_directory("stretch_core")
    rosbridge_server_dir = get_package_share_directory("rosbridge_server")

    # ros2 launch stretch_core stretch_driver.launch.py
    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stretch_core_dir, "launch", "stretch_driver.launch.py")
        )
    )

    # ros2 launch stretch_core d435i_low_resolution.launch.py
    d435i_low_resolution_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stretch_core_dir, "launch", "d435i_low_resolution.launch.py")
        )
    )


    # ros2 launch stretch_core d405_basic.launch.py
    d405_basic_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(stretch_core_dir, "launch", "d405_basic.launch.py")
        )
    )

    # ros2 launch rosbridge_server rosbridge_websocket_launch.xml
    rosbridge_websocket_launch = IncludeLaunchDescription(
        XMLLaunchDescriptionSource(
            os.path.join(
                rosbridge_server_dir, "launch", "rosbridge_websocket_launch.xml"
            )
        )
    )

    # launch all 3  launch files
    return LaunchDescription(
        [stretch_driver_launch, d435i_low_resolution_launch, d405_basic_launch, rosbridge_websocket_launch]
    )
