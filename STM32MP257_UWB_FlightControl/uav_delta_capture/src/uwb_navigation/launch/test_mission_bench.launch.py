#!/usr/bin/env python3
"""Real hardware bench test: FCU/UWB/local-pose preflight + ARM + Z velocity profile."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


def generate_launch_description():
    share_dir = get_package_share_directory('uwb_navigation')

    test_mission_node = Node(
        package='uwb_navigation',
        executable='test_mission_node.py',
        name='test_mission_node',
        parameters=[os.path.join(share_dir, 'test_mission_bench.yaml')],
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
            parameters=[{'skip_ekf_check': True, 'vel_timeout_sec': 0.5}],
        ),
        Node(
            package='fcu_bridge',
            executable='flight_state_machine_node',
            name='flight_state_machine_node',
        ),
        Node(
            package='uwb_driver',
            executable='uwb_aoa_driver_node',
            name='uwb_aoa_driver_node',
            parameters=[{'serial_port': '/dev/ttyUSB0', 'serial_baud': 115200}],
        ),
        test_mission_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=test_mission_node,
                on_exit=[
                    EmitEvent(
                        event=Shutdown(reason='bench test_mission_node completed')
                    )
                ],
            )
        ),
    ])
