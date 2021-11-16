import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    ur_prefix = get_package_share_directory('ur_bringup')
    igtl_prefix = get_package_share_directory('ros2_igtl_bridge')
    
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ur_prefix, '/launch/ur_control.launch.py']),
            launch_arguments={'ur_type': 'ur10e', 'robot_ip': 'yyy.yyy.yyy.yyy', 'use_fake_hardware': 'true', 'launch_rviz': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ur_prefix, '/launch/ur_moveit.launch.py']),
            launch_arguments={'ur_type': 'ur10e', 'robot_ip': 'yyy.yyy.yyy.yyy', 'use_fake_hardware': 'true', 'launch_rviz': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([igtl_prefix, '/launch/ur_moveit.launch.py']),
            launch_arguments={'ur_type': 'ur10e', 'robot_ip': 'yyy.yyy.yyy.yyy', 'use_fake_hardware': 'true', 'launch_rviz': 'true'}.items()),
        ])
