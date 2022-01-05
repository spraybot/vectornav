import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    this_dir = get_package_share_directory('vectornav')

    playback_mode = DeclareLaunchArgument('playback', default_value='False')
    
    start_vectornav_cmd = Node(
        package='vectornav', 
        executable='vectornav',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')],
        condition=UnlessCondition(LaunchConfiguration("playback")))
    
    start_vectornav_sensor_msgs_cmd = Node(
        package='vectornav', 
        executable='vn_sensor_msgs',
        output='screen',
        parameters=[os.path.join(this_dir, 'config', 'vectornav.yaml')])

    # Create the launch description and populate
    ld = LaunchDescription()

    ld.add_action(playback_mode)
    
    if playback_mode:
        ld.add_action(start_vectornav_cmd)
    ld.add_action(start_vectornav_sensor_msgs_cmd)

    return ld
