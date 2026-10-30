import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """建图阶段：slam_toolbox（Karto + SPA 位姿图）+ TF 桥接 + 精度评估。

    注意：slam_toolbox 是 lifecycle 节点，必须走它自带的 online_async_launch.py，
    由 launch 事件完成 configure -> activate；直接启动 executable 会停在 unconfigured，
    表现为「节点在、但既不订阅 /scan 也不发 /map」。
    """
    slam_pkg = get_package_share_directory('parking_slam')
    toolbox_pkg = get_package_share_directory('slam_toolbox')
    params = os.path.join(slam_pkg, 'config', 'slam_toolbox_mapping.yaml')

    eval_csv = LaunchConfiguration('eval_csv')

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(toolbox_pkg, 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': params,
            'autostart': 'true',
        }.items(),
    )

    bridge = Node(
        package='parking_slam',
        executable='tf_bridge',
        name='slam_tf_bridge',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    evaluator = Node(
        package='parking_slam',
        executable='slam_eval',
        name='slam_eval',
        output='screen',
        parameters=[{'use_sim_time': True, 'csv': eval_csv}],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'eval_csv', default_value=os.path.expanduser('~/apa_ws/logs/ate.csv')),
        slam,
        bridge,
        evaluator,
    ])
