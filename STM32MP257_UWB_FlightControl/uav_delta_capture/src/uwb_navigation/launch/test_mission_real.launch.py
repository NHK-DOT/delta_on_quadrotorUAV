#!/usr/bin/env python3
"""Backward-compatible bench test entry. MAVROS must be started separately."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('uwb_navigation'),
                    'test_mission_bench.launch.py',
                ])
            )
        )
    ])
