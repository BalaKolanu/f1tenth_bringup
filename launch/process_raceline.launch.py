#!/usr/bin/env python3

import glob
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, RegisterEventHandler, Shutdown
from launch.event_handlers import OnProcessExit
from launch.launch_context import LaunchContext
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SOURCE_REPO_ROOT = '/home/nvidia/f1tenth_ws/src/f1tenth_bringup'
SOURCE_MAP_DIR = os.path.join(SOURCE_REPO_ROOT, 'maps')
SOURCE_MANUAL_PATH_DIR = os.path.join(SOURCE_REPO_ROOT, 'manual_paths')
SOURCE_LANE_DIR = os.path.join(SOURCE_REPO_ROOT, 'lanes')
SOURCE_PATH_DIR = os.path.join(SOURCE_REPO_ROOT, 'paths')
SOURCE_ARCHIVE_DIR = os.path.join(SOURCE_REPO_ROOT, 'old_paths')
SOURCE_PREVIEW_PNG = os.path.join(SOURCE_LANE_DIR, 'lane_preview.png')


def _failed(event) -> bool:
    return getattr(event, 'returncode', 1) != 0


def _select_seed_directory(manual_path_directory: str, fallback_path_directory: str) -> str:
    manual_paths = sorted(
        path for path in glob.glob(os.path.join(manual_path_directory, '*.csv'))
        if os.path.isfile(path)
    )
    return manual_path_directory if manual_paths else fallback_path_directory


def _configure_pipeline(context: LaunchContext):
    manual_path_directory = LaunchConfiguration('manual_path_directory').perform(context)
    fallback_path_directory = LaunchConfiguration('fallback_path_directory').perform(context)
    seed_directory = _select_seed_directory(manual_path_directory, fallback_path_directory)
    seed_name = 'manual path' if seed_directory == manual_path_directory else 'fallback raceline'

    generate_lanes_node = Node(
        package='raceline_generation',
        executable='generate_lanes',
        name='generate_lanes',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'map_directory': LaunchConfiguration('map_directory'),
            'manual_path_directory': seed_directory,
            'output_directory': LaunchConfiguration('lane_directory'),
            'lane_preview_png': LaunchConfiguration('preview_png'),
            'publish_lanes': False,
        }],
    )

    process_raceline_node = Node(
        package='raceline_generation',
        executable='process_raceline',
        name='process_raceline',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'map_directory': LaunchConfiguration('map_directory'),
            'manual_path_directory': seed_directory,
            'lane_directory': LaunchConfiguration('lane_directory'),
            'output_directory': LaunchConfiguration('output_directory'),
            'archive_directory': LaunchConfiguration('archive_directory'),
            'preview_png': LaunchConfiguration('preview_png'),
            'use_lane_bounds': True,
            'save_preview': True,
        }],
    )

    def on_lanes_done(event, _context: LaunchContext):
        if _failed(event):
            return [
                LogInfo(msg='generate_lanes failed; stopping raceline pipeline.'),
                Shutdown(reason='generate_lanes failed'),
            ]
        return [
            LogInfo(msg='Lane generation complete; starting raceline optimization.'),
            process_raceline_node,
        ]

    def on_raceline_done(event, _context: LaunchContext):
        if _failed(event):
            return [
                LogInfo(msg='process_raceline failed; stopping raceline pipeline.'),
                Shutdown(reason='process_raceline failed'),
            ]
        return [LogInfo(msg='Processed raceline is ready.')]

    return [
        LogInfo(msg=f'Using {seed_name} seed directory: {seed_directory}'),
        generate_lanes_node,
        RegisterEventHandler(OnProcessExit(
            target_action=generate_lanes_node,
            on_exit=on_lanes_done,
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=process_raceline_node,
            on_exit=on_raceline_done,
        )),
    ]


def generate_launch_description():
    arguments = [
        DeclareLaunchArgument('map_directory', default_value=SOURCE_MAP_DIR),
        DeclareLaunchArgument('manual_path_directory', default_value=SOURCE_MANUAL_PATH_DIR),
        DeclareLaunchArgument('fallback_path_directory', default_value=SOURCE_PATH_DIR),
        DeclareLaunchArgument('lane_directory', default_value=SOURCE_LANE_DIR),
        DeclareLaunchArgument('output_directory', default_value=SOURCE_PATH_DIR),
        DeclareLaunchArgument('archive_directory', default_value=SOURCE_ARCHIVE_DIR),
        DeclareLaunchArgument('preview_png', default_value=SOURCE_PREVIEW_PNG),
    ]
    return LaunchDescription(arguments + [
        LogInfo(msg='f1tenth_bringup raceline pipeline: generate_lanes -> process_raceline'),
        OpaqueFunction(function=_configure_pipeline),
    ])
