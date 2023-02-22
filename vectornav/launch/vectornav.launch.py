import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    ld = LaunchDescription()
    this_dir = get_package_share_directory('vectornav')

    # TODO: Add support for playback mode
    playback_mode = DeclareLaunchArgument('playback', default_value='False')

    vectornav_node = ComposableNode(
        package='vectornav', 
        plugin='Vectornav',
        name='vectornav',
        parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')])
        # condition=UnlessCondition(LaunchConfiguration("playback")))
    
    vectornav_sensor_msgs_node = ComposableNode(
        package='vectornav', 
        plugin='VnSensorMsgs',
        name='vn_sensor_msgs',
        parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')],
        remappings=[('vectornav/imu', 'imu/data'),
                    ('vectornav/gnss/ins', 'gps/fix'),
                    ('vectornav/gnss/raw', 'gps/fix/raw')])

    container = ComposableNodeContainer(
            name='vectornav_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                vectornav_node, vectornav_sensor_msgs_node
            ],
            output='screen',
    )
    ld.add_action(container)

    lifecycle_manager = Node(
        package='lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_sensors',
        output='screen',
        parameters=[
            {'autostart': True},
            {'node_names': ['vectornav'], 'node_transitions': ['activate'], 'bond_timeout': 0.0},
        ])
    ld.add_action(lifecycle_manager)

    return ld
