#!/usr/bin/env python3
"""一键启动 mock 测试: 4 个 mock 桥接节点 + test_mission_node (test_mission_mock.yaml)"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('uwb_navigation')

    return LaunchDescription([
        # ── 模拟飞控 ──
        Node(
            package='fcu_bridge', executable='mock_mavros_pose_node', name='mock_mavros_pose_node',
            parameters=[{'mock_altitude': 0.0}],
        ),
        Node(
            package='fcu_bridge', executable='fcu_state_node', name='fcu_state_node',
            parameters=[{'use_mock': True, 'mock_armed': True, 'mock_altitude': 0.0}],
        ),
        Node(
            package='fcu_bridge', executable='flight_commander_node', name='flight_commander_node',
            parameters=[{'use_mock': True, 'vel_timeout_sec': 0.5}],
        ),
        Node(
            package='fcu_bridge', executable='fcu_link_monitor_node', name='fcu_link_monitor_node',
        ),
        Node(
            package='fcu_bridge', executable='flight_state_machine_node', name='flight_state_machine_node',
        ),

        # ── 测试任务节点 ──
        Node(
            package='uwb_navigation',
            executable='test_mission_node.py',
            name='test_mission_node',
            parameters=[os.path.join(share_dir, 'test_mission_mock.yaml')],
            output='screen',
        ),
    ])
