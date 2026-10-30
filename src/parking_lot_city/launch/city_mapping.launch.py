"""建图/仿真基础环境：Gazebo 场景 + ros_gz 桥 + slam_toolbox + TF 桥 + 自车生成。

[同学 C 改造]
* 自车不再写死在 world 里，改为用 `ros_gz_sim create` 按 spawn_x/y/yaw 生成，
  因此主路上任意位置都能作为演示起点（`scripts/run_demo.sh` 的 SPAWN_* 环境变量）；
* 航点驱动默认关闭（`drive_patrol:=false`）—— 现在由规划层自己做「全局路线 +
  泊车」，需要只建图巡场时再打开；
* 仍保留 slam_toolbox 官方 online_async launch（lifecycle 自动激活）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 巡场路线（世界坐标；绕停车场中央绿化带走一圈）
PATROL_WAYPOINTS = "-9.5,-1;-9.5,-18;9.5,-18;9.5,-1;-9.5,-1"


def generate_launch_description():
    pkg = get_package_share_directory("parking_lot_city")
    world = os.path.join(pkg, "worlds", "city_parking_lot.sdf")
    models = os.path.join(pkg, "models")
    bridge_cfg = os.path.join(pkg, "config", "gz_bridge.yaml")
    rviz_cfg = os.path.join(pkg, "rviz", "parking_demo.rviz")
    ego_sdf = os.path.join(models, "ego_car", "model.sdf")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    drive_patrol = LaunchConfiguration("drive_patrol")
    patrol_speed = LaunchConfiguration("patrol_speed")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    # ---------------- Gazebo 进程编排 ----------------
    # headless：只跑服务端。
    # GUI 模式：**先只跑服务端**，等自车生成之后再单独拉起 GUI（`gz sim -g`）。
    #
    # 为什么不能像以前那样一条 `gz sim -r <world>` 同时起 server+GUI：
    # 实测发现，GUI 在连上服务器的那一刻只取一次完整场景快照
    # （/world/<world>/scene/info），之后靠增量消息更新；而自车是用
    # `ros_gz_sim create` 在运行期插进去的实体。增量更新一旦没被应用，
    # 服务器里明明有 ego_vehicle（pose/info、scene/info 里几何和材质都齐全，
    # id 1182~1209 一共 14 个 visual），GUI 里却永远看不到它 ——
    # 现象就是「场景里只有一台蓝色干扰车，找不到自车」。
    # 让 GUI 晚 13 s 再连上来，它拿到的首帧快照里就包含自车。
    gz_server = ExecuteProcess(
        cmd=["gz", "sim", "-s", "-r", "-v", "2", world],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": models},
    )
    gz_gui = ExecuteProcess(
        cmd=["gz", "sim", "-g", "-v", "2"],
        output="screen",
        additional_env={"GZ_SIM_RESOURCE_PATH": models},
        condition=UnlessCondition(headless),
    )

    # 运行时生成自车：主路任意位置都可以作为起点
    spawn_ego = Node(
        package="ros_gz_sim",
        executable="create",
        name="spawn_ego",
        output="screen",
        arguments=["-name", "ego_vehicle", "-file", ego_sdf,
                   "-x", spawn_x, "-y", spawn_y, "-z", "0.03", "-Y", spawn_yaw],
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="gz_bridge",
        output="screen",
        parameters=[{"config_file": bridge_cfg}],
    )

    toolbox_pkg = get_package_share_directory("slam_toolbox")
    slam_pkg = get_package_share_directory("parking_slam")
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(toolbox_pkg, "launch", "online_async_launch.py")),
        launch_arguments={
            "use_sim_time": "true",
            "slam_params_file": os.path.join(slam_pkg, "config",
                                             "slam_toolbox_mapping.yaml"),
            "autostart": "true",
        }.items(),
    )

    tf_bridge = Node(
        package="parking_slam",
        executable="tf_bridge",
        name="slam_tf_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    driver = Node(
        package="parking_slam",
        executable="demo_driver",
        name="ego_waypoint_driver",
        output="screen",
        parameters=[{
            "odom_topic": "/odom_gt",
            "odom_world": True,
            "spawn_x": -9.5,
            "spawn_y": -1.0,
            "spawn_yaw": -1.5708,
            "cmd_topic": "/cmd_vel_driver",
            "status_topic": "/driver_status",
            "waypoints": PATROL_WAYPOINTS,
            "speed": ParameterValue(patrol_speed, value_type=float),
            "lookahead": 2.6,
            "fillet_radius": 4.0,
            "path_step": 0.4,
            "max_steer": 0.60,
            "loop": False,
        }],
        condition=IfCondition(drive_patrol),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_cfg],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("drive_patrol", default_value="false"),
        DeclareLaunchArgument("patrol_speed", default_value="1.2"),
        # 默认：主路东行外侧车道，停车场入口以西约 45 m
        DeclareLaunchArgument("spawn_x", default_value="-45.0"),
        DeclareLaunchArgument("spawn_y", default_value="15.65"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        gz_server,
        TimerAction(period=4.0, actions=[bridge]),
        TimerAction(period=5.0, actions=[spawn_ego]),
        TimerAction(period=7.0, actions=[slam, tf_bridge]),
        TimerAction(period=10.0, actions=[driver]),
        TimerAction(period=12.0, actions=[rviz]),
        # GUI 必须晚于 spawn_ego：它只在连接时取一次完整场景快照
        TimerAction(period=13.0, actions=[gz_gui]),
    ])
