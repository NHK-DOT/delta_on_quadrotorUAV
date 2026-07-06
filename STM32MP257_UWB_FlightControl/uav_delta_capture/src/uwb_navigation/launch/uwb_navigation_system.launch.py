#!/usr/bin/env python3
"""Full UWB mission system entry.

MAVROS is still expected to be started separately with the FCU URL.
"""
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
                    'test_mission_real_full.launch.py',
                ])
            )
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('vision_bridge'),
                    'vision_transform.launch.py',
                ])
            )
        ),
    ])
