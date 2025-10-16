#!/usr/bin/env python3
"""
Launch file for ORCA hardware interface node.

Usage:
    ros2 launch orca_hardware_interface orca_hardware.launch.py
    
With custom model path:
    ros2 launch orca_hardware_interface orca_hardware.launch.py model_path:=/path/to/model
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for ORCA hardware node."""
    
    # Declare launch arguments
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to ORCA hand model directory (leave empty for default)'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='90.0',
        description='Control loop frequency in Hz (matches OpenTeach rate)'
    )
    
    auto_connect_arg = DeclareLaunchArgument(
        'auto_connect',
        default_value='true',
        description='Automatically connect to hardware on startup'
    )
    
    auto_calibrate_arg = DeclareLaunchArgument(
        'auto_calibrate',
        default_value='false',
        description='Automatically calibrate on startup (takes time!)'
    )
    
    # Create node
    orca_hardware_node = Node(
        package='orca_hardware_interface',
        executable='orca_hardware_node',
        name='orca_hardware_node',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'control_frequency': LaunchConfiguration('control_frequency'),
            'auto_connect': LaunchConfiguration('auto_connect'),
            'auto_calibrate': LaunchConfiguration('auto_calibrate'),
        }]
    )
    
    return LaunchDescription([
        model_path_arg,
        control_frequency_arg,
        auto_connect_arg,
        auto_calibrate_arg,
        orca_hardware_node,
    ])
