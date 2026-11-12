#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    f1tenth_stack_share = get_package_share_directory('f1tenth_stack')
    bringup_launch = os.path.join(f1tenth_stack_share, 'launch', 'bringup_launch.py')
    default_joy_config = os.path.join(f1tenth_stack_share, 'config', 'joy_teleop.yaml')
    default_vesc_config = os.path.join(f1tenth_stack_share, 'config', 'vesc.yaml')
    default_sensors_config = os.path.join(f1tenth_stack_share, 'config', 'sensors.yaml')
    default_mux_config = os.path.join(f1tenth_stack_share, 'config', 'mux.yaml')
    default_maps_directory = '/home/nvidia/f1tenth_ws/src/f1tenth_bringup/maps'

    joy_config = DeclareLaunchArgument('joy_config', default_value=default_joy_config)
    vesc_config = DeclareLaunchArgument('vesc_config', default_value=default_vesc_config)
    sensors_config = DeclareLaunchArgument('sensors_config', default_value=default_sensors_config)
    mux_config = DeclareLaunchArgument('mux_config', default_value=default_mux_config)
    save_button_index = DeclareLaunchArgument(
        'save_button_index', default_value='3',
        description='Joystick button index used to save the map.',
    )
    reset_button_index = DeclareLaunchArgument(
        'reset_button_index', default_value='0',
        description='Joystick button index used to clear the map and restart SLAM.',
    )
    maps_directory = DeclareLaunchArgument(
        'maps_directory', default_value=default_maps_directory,
        description='Directory where generated map files are saved.',
    )
    map_basename = DeclareLaunchArgument(
        'map_basename', default_value='racetrack',
        description='Base name for generated map files.',
    )

    stack_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'joy_config': LaunchConfiguration('joy_config'),
            'vesc_config': LaunchConfiguration('vesc_config'),
            'sensors_config': LaunchConfiguration('sensors_config'),
            'mux_config': LaunchConfiguration('mux_config'),
        }.items(),
    )

    mapping_session_manager = Node(
        package='f1tenth_bringup',
        executable='mapping_session_manager',
        name='mapping_session_manager',
        output='screen',
        parameters=[{
            'joy_topic': '/joy',
            'save_button_index': LaunchConfiguration('save_button_index'),
            'reset_button_index': LaunchConfiguration('reset_button_index'),
            'maps_directory': LaunchConfiguration('maps_directory'),
            'map_basename': LaunchConfiguration('map_basename'),
        }],
    )

    return LaunchDescription([
        joy_config, vesc_config, sensors_config, mux_config,
        save_button_index, reset_button_index, maps_directory, map_basename,
        stack_bringup, mapping_session_manager,
    ])
