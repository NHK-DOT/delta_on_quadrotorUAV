#!/usr/bin/env python3
"""Launch read-only interactive UWB calibration recorder."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("tag_height_m", default_value="0.0"),
        DeclareLaunchArgument("sample_window_sec", default_value="3.0"),
        DeclareLaunchArgument("rangefinder_topic", default_value="/mavros/rangefinder_pub"),
        DeclareLaunchArgument("rangefinder_timeout_sec", default_value="1.0"),
        DeclareLaunchArgument("output_dir", default_value="/tmp"),
        Node(
            package="uwb_driver",
            executable="uwb_aoa_driver_node",
            name="uwb_aoa_driver_node",
            parameters=[{
                "serial_port": "/dev/ttyUSB0",
                "serial_baud": 115200,
                "uwb_aoa_topic": "uwb_aoa/data",
            }],
            output="screen",
        ),
        Node(
            package="uwb_navigation",
            executable="uwb_calibration_recorder.py",
            name="uwb_calibration_recorder",
            parameters=[{
                "uwb_aoa_topic": "uwb_aoa/data",
                "rangefinder_topic": LaunchConfiguration("rangefinder_topic"),
                "rangefinder_timeout_sec": LaunchConfiguration("rangefinder_timeout_sec"),
                "tag_height_m": LaunchConfiguration("tag_height_m"),
                "uwb_azimuth_offset_deg": 0.0,
                "uwb_mount_pitch_down_deg": -45.0,
                "uwb_forward_sign": 1.0,
                "uwb_lateral_sign": -1.0,
                "uwb_min_body_elevation_deg": 8.0,
                "uwb_approach_front_sector_deg": 65.0,
                "uwb_capture_front_sector_deg": 30.0,
                "sample_window_sec": LaunchConfiguration("sample_window_sec"),
                "live_print_period_sec": 1.0,
                "output_dir": LaunchConfiguration("output_dir"),
            }],
            output="screen",
            emulate_tty=True,
        ),
    ])
