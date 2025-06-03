import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.launch_context import LaunchContext
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
import rclpy


def generate_launch_description():
    # Navigation stack
    logger = rclpy.logging.get_logger('navigation_launch')

    stretch_core_path = get_package_share_directory('stretch_core')
    stretch_navigation_path = get_package_share_directory('stretch_nav2')
    navigation_bringup_path = get_package_share_directory('nav2_bringup')
    rosbridge_server_dir = get_package_share_directory("rosbridge_server")
    web_teleop_dir = get_package_share_directory("web_teleop")

    # stretch_core driver

    stretch_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/stretch_driver.launch.py']),
        launch_arguments={'mode': 'navigation', 'broadcast_odom_tf': 'True'}.items())

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

    stretch_aruco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([web_teleop_dir, '/launch/stretch_aruco.launch.py']),
    )

    # ros2 run web_teleop aruco_tag_locator
    tag_locator_node = Node(
        package='web_teleop',
        executable='aruco_tag_locator',
        name='aruco_tag_locator',
        output='screen',
    )
    ########################################3
    # Navigation launch
    teleop_type_param = DeclareLaunchArgument(
        'teleop_type', default_value="keyboard", description="how to teleop ('keyboard', 'joystick' or 'none')")
    
    use_sim_time_param = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation/Gazebo clock')

    autostart_param = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Whether to autostart lifecycle nodes on launch')

    map_path_param = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(stretch_navigation_path,
                                   'map', '/home/hello-robot/ada_pet_capstone/src/ada_pet_capstone/web_teleop/config/capstone_room.yaml'),
        description='Full path to the map.yaml file to use for navigation')

    # Error out if the map file does not exist
    def map_file_check(context: LaunchContext):
        map_path = LaunchConfiguration('map').perform(context)
        if not os.path.exists(map_path):
            msg='Map file not found in given path: {}'.format(map_path)
            logger.error(msg)
            raise FileNotFoundError(msg)
        if not map_path.endswith('.yaml'):
            msg = 'Map file is not a yaml file: {}'.format(map_path)
            logger.error(msg)
            raise FileNotFoundError(msg)
        
    map_path_check_action = OpaqueFunction(function=map_file_check)

    params_file_param = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(stretch_navigation_path, 'config', 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')
        
    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])


    rplidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_core_path, '/launch/rplidar.launch.py']))

    base_teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/teleop_twist.launch.py']),
        launch_arguments={'teleop_type': LaunchConfiguration('teleop_type')}.items())

    navigation_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([stretch_navigation_path, '/launch/bringup_launch.py']),
        launch_arguments={'use_sim_time': LaunchConfiguration('use_sim_time'), 
                          'autostart': LaunchConfiguration('autostart'),
                          'map': LaunchConfiguration('map'),
                          'params_file': LaunchConfiguration('params_file'),
                          'use_rviz': LaunchConfiguration('use_rviz')}.items())

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([navigation_bringup_path, '/launch/rviz_launch.py']),
        condition=IfCondition(LaunchConfiguration('use_rviz')))


    return LaunchDescription(
        [
            stretch_driver_launch,
            rosbridge_websocket_launch,
            multi_cam_launch,
            play_sound_node,
            stretch_aruco_launch,
            tag_locator_node,

            teleop_type_param,
            use_sim_time_param,
            autostart_param,
            map_path_param,
            params_file_param,
            rviz_param,
            rplidar_launch,
            base_teleop_launch,
            navigation_bringup_launch,
            rviz_launch,
            map_path_check_action,
        ]
    )
