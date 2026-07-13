#!/usr/bin/env python3
"""Prop-on GUIDED takeoff, local-position waypoint, return, hover, and LAND test."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('uwb_navigation')

    test_mission_node = Node(
        package='uwb_navigation',
        executable='test_mission_node.py',
        name='test_mission_node',
        parameters=[os.path.join(share_dir, 'test_mission_takeoff_waypoint_return_land.yaml')],
        output='screen',
    )

    set_body_frame = ExecuteProcess(
        cmd=[
            'ros2',
            'param',
            'set',
            '/mavros/setpoint_velocity',
            'mav_frame',
            'BODY_NED',
        ],
        output='screen',
    )

    return LaunchDescription([
        Node(
            package='fcu_bridge',
            executable='fcu_state_node',
            name='fcu_state_node',
        ),
        Node(
            package='fcu_bridge',
            executable='fcu_link_monitor_node',
            name='fcu_link_monitor_node',
        ),
        Node(
            package='fcu_bridge',
            executable='flight_commander_node',
            name='flight_commander_node',
            parameters=[{
                'skip_ekf_check': True,
                'vel_timeout_sec': 0.5,
                'takeoff_target_clearance_m': 0.2,
            }],
        ),
        Node(
            package='fcu_bridge',
            executable='flight_state_machine_node',
            name='flight_state_machine_node',
        ),
        set_body_frame,
        RegisterEventHandler(
            OnProcessExit(
                target_action=set_body_frame,
                on_exit=[test_mission_node],
            )
        ),
        RegisterEventHandler(
            OnProcessExit(
                target_action=test_mission_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason='takeoff_waypoint_return_land test_mission_node completed')
                    )
                ],
            )
        ),
    ])
