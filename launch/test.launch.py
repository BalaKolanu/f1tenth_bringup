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
    particle_filter_share = get_package_share_directory('particle_filter')

    bringup_launch = os.path.join(f1tenth_stack_share, 'launch', 'bringup_launch.py')
    default_joy_config = os.path.join(f1tenth_stack_share, 'config', 'joy_teleop.yaml')
    default_vesc_config = os.path.join(f1tenth_stack_share, 'config', 'vesc.yaml')
    default_sensors_config = os.path.join(f1tenth_stack_share, 'config', 'sensors.yaml')
    default_mux_config = os.path.join(f1tenth_stack_share, 'config', 'mux.yaml')
    default_localize_config = os.path.join(particle_filter_share, 'config', 'localize.yaml')
    default_map_yaml = os.path.join(particle_filter_share, 'maps', 'levine.yaml')
    default_raceline = '/home/nvidia/f1tenth_ws/src/f1tenth_bringup/paths/raceline.csv'

    joy_config = DeclareLaunchArgument(
        'joy_config', default_value=default_joy_config,
        description='Joystick and teleoperation configuration.')
    vesc_config = DeclareLaunchArgument(
        'vesc_config', default_value=default_vesc_config,
        description='VESC driver and odometry configuration.')
    sensors_config = DeclareLaunchArgument(
        'sensors_config', default_value=default_sensors_config,
        description='LiDAR and sensor configuration.')
    mux_config = DeclareLaunchArgument(
        'mux_config', default_value=default_mux_config,
        description='Ackermann command multiplexer configuration.')
    localize_config = DeclareLaunchArgument(
        'localize_config', default_value=default_localize_config,
        description='Particle-filter parameters.')
    map_yaml = DeclareLaunchArgument(
        'map_yaml', default_value=default_map_yaml,
        description='Map YAML consumed by nav2_map_server.')
    raceline_csv = DeclareLaunchArgument(
        'raceline_csv', default_value=default_raceline,
        description='Raceline waypoint CSV consumed by the MPC controller.')

    vehicle_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch),
        launch_arguments={
            'joy_config': LaunchConfiguration('joy_config'),
            'vesc_config': LaunchConfiguration('vesc_config'),
            'sensors_config': LaunchConfiguration('sensors_config'),
            'mux_config': LaunchConfiguration('mux_config'),
        }.items(),
    )

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen', parameters=[{
            'yaml_filename': LaunchConfiguration('map_yaml'),
            'topic': 'map', 'frame_id': 'map', 'use_sim_time': False,
        }],
    )

    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen', parameters=[{
            'use_sim_time': False, 'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    particle_filter = Node(
        package='particle_filter', executable='particle_filter',
        name='particle_filter', output='screen',
        parameters=[LaunchConfiguration('localize_config')],
    )

    mpc = Node(
        package='f1tenth_controllers', executable='mpc_controller',
        name='mpc_controller', output='screen', parameters=[{
            'odom_topic': '/odom',
            'drive_topic': 'drive',
            'cmd_topic': 'drive',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'use_tf_pose_feedback': True,
            'waypoint_csv': LaunchConfiguration('raceline_csv'),
        }],
    )

    return LaunchDescription([
        joy_config, vesc_config, sensors_config, mux_config,
        localize_config, map_yaml, raceline_csv,
        vehicle_bringup, map_server, map_lifecycle_manager,
        particle_filter, mpc,
    ])
