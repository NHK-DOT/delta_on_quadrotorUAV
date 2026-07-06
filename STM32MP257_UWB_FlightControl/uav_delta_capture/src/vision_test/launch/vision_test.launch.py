from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    bench_default = os.path.join(
        get_package_share_directory('vision_test'),
        'config',
        'vision_test.yaml'
    )

    bench_params = LaunchConfiguration('bench_params')
    model_path = LaunchConfiguration('model_path')
    input_size = LaunchConfiguration('input_size')
    use_npu = LaunchConfiguration('use_npu')
    num_iterations = LaunchConfiguration('num_iterations')
    mode = LaunchConfiguration('mode')
    report_path = LaunchConfiguration('report_path')
    camera_device = LaunchConfiguration('camera_device')
    camera_width = LaunchConfiguration('camera_width')
    camera_height = LaunchConfiguration('camera_height')
    save_frames = LaunchConfiguration('save_frames')

    return LaunchDescription([
        DeclareLaunchArgument('bench_params', default_value=bench_default),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('input_size', default_value='320'),
        DeclareLaunchArgument('use_npu', default_value='true'),
        DeclareLaunchArgument('num_iterations', default_value='100'),
        DeclareLaunchArgument('mode', default_value='synthetic'),
        DeclareLaunchArgument('report_path', default_value='/tmp/vision_test_report.txt'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video7'),
        DeclareLaunchArgument('camera_width', default_value='640'),
        DeclareLaunchArgument('camera_height', default_value='480'),
        DeclareLaunchArgument('save_frames', default_value='false'),

        Node(
            package='vision_test',
            executable='vision_test_node',
            name='vision_test_node',
            output='screen',
            parameters=[
                bench_params,
                {
                    'model_path': model_path,
                    'input_size': input_size,
                    'use_npu': use_npu,
                    'num_iterations': num_iterations,
                    'mode': mode,
                    'report_path': report_path,
                    'camera_device': camera_device,
                    'camera_width': camera_width,
                    'camera_height': camera_height,
                    'save_frames': save_frames,
                },
            ],
        ),
    ])
