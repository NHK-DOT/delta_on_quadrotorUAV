from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 获取配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('vision_bridge'),
        'config'
    )
    config_file = os.path.join(config_dir, 'vision_transform.yaml')

    return LaunchDescription([
        # 声明参数
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file,
            description='Path to config file'
        ),

        # vision_transform_node
        Node(
            package='vision_bridge',
            executable='vision_transform_node',
            name='vision_transform_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file')
            ],
        ),
    ])
