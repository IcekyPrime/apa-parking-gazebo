"""全链路一键演示：主路出发 → 全局路线 → 选位 → 泊车（含多点揉库）。

组成
----
* A：Gazebo 场景、ros_gz 桥、slam_toolbox、tf_bridge（/vehicle_pose）、运行时生成自车
* B：detection_tracking → trajectory_prediction → risk_assessment（/risk_level）
* C：parking_planner（全局路线 + 选位 + Hybrid A* + 跟踪）、cmd_mux、parking_eval
* 场景干扰物：scene_agents 的主路循环车（traffic_driver）与抢位干扰车（parker）

用法
----
    ros2 launch parking_planner parking_demo.launch.py
    # 从主路任意位置出发：
    ros2 launch parking_planner parking_demo.launch.py spawn_x:=-60 spawn_y:=15.65
    # 指定车位（场内垂直/平行、四条道路路边均可）：
    ros2 launch parking_planner parking_demo.launch.py goal_stall:=SW3
    # 抢位场景（干扰车匀速停进 E8；goal_stall 留空让选位器自动选，
    # 它应当在激光里看到 E8 被占用后改选别的车位）：
    ros2 launch parking_planner parking_demo.launch.py use_parker:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    city_pkg = get_package_share_directory("parking_lot_city")
    planner_pkg = get_package_share_directory("parking_planner")

    headless = LaunchConfiguration("headless")
    use_rviz = LaunchConfiguration("use_rviz")
    drive_patrol = LaunchConfiguration("drive_patrol")
    use_prediction = LaunchConfiguration("use_prediction")
    goal_stall = LaunchConfiguration("goal_stall")
    stall_region = LaunchConfiguration("stall_region")
    prefer_stalls = LaunchConfiguration("prefer_stalls")
    max_speed = LaunchConfiguration("max_speed")
    eval_csv = LaunchConfiguration("eval_csv")
    model_path = LaunchConfiguration("model_path")
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_yaw = LaunchConfiguration("spawn_yaw")

    city = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(city_pkg, "launch", "city_mapping.launch.py")),
        launch_arguments={
            "headless": headless,
            "use_rviz": use_rviz,
            "drive_patrol": drive_patrol,
            "spawn_x": spawn_x,
            "spawn_y": spawn_y,
            "spawn_yaw": spawn_yaw,
        }.items(),
    )

    # ---------------- 场景干扰物 ----------------
    # 两辆干扰车都写在 world 里（见 tools/build_world.py），这里只负责驱动。
    # 路线是**全地图闭环**：主干道 → 斑马线 → 停车场内绕圈 → 三条街 → 回主干道；
    # 开局用 set_pose 随机落到闭环某处。
    # [同学 C] 让行判据改成「前方走廊矩形 ∩ 对方车身矩形」（见 path_agent._blocked），
    # stop_radius 7.5 → 9.0：两台 4.35 m 长的车横着交会时，中心距 3.1 m 就已经
    # 碰上了，7.5 m 只剩 4.4 m 的反应距离，2 m/s 下只有 2.2 s。9 m 给到 3.7 s。
    # 9.0 → 10.5（2026-10-28 W5 真机）：自车在 W5 揉库期间按既定轨迹倒车（机动
    # 执行段不重规划），蓝色干扰车 9 m 走廊仍探进过道，两车在自车倒车弧上相撞
    # （自车左前角 × 干扰车右前角）。走廊加长 1.5 m，让它在更远处就停车让行。
    VEH_TOPICS = "/odom_gt,/traffic/odom_gt,/traffic2/odom_gt"
    traffic = Node(
        package="scene_agents",
        executable="traffic_driver",
        name="traffic_driver",
        output="screen",
        parameters=[{
            "world": "city_parking_lot",
            "entity": "traffic_car",
            "odom_topic": "/traffic/odom_gt",
            "cmd_topic": "/traffic/cmd_vel",
            "speed": 2.0,
            "stop_radius": 10.5,
            # [本轮修复 · 建议 5a] 死锁时**一次退出让行半径**。
            # 先试过 unblock_back=4.0 / unblock_max=6，真机 B3 的日志（a1）显示
            # 干扰车老老实实退了 6 轮 × 4 m = 24 m，自车**仍然一寸没动** ——
            # 因为每退 4 m 时它依然落在自车 4.5 m 走廊判据 / 自己 10.5 m 让行
            # 半径的耦合区里，退完又停、停了又退，节奏永远追不上。
            # 所以一次就要退到让行半径之外：12 m > 10.5 m。最多 3 次（36 m）。
            "unblock_back": 12.0,
            "unblock_max": 3,
            # [本轮 · B3/SE1] 避障时间增长：等 15 s 才考虑倒退，
            # 退完再原地停留 15 s 让自车通过（不再退完马上前进再次迎面顶住）。
            "unblock_wait": 15.0,
            "unblock_hold": 15.0,
            "vehicles": VEH_TOPICS,
            "pedestrians": "/pedestrians",
        }],
    )
    traffic_b = Node(
        package="scene_agents",
        executable="traffic_driver_b",
        name="traffic_driver_b",
        output="screen",
        parameters=[{
            "world": "city_parking_lot",
            "entity": "traffic_car_b",
            "odom_topic": "/traffic2/odom_gt",
            "cmd_topic": "/traffic2/cmd_vel",
            "speed": 1.8,
            "stop_radius": 10.5,
            # 与 traffic_driver 同口径（一次退 12 m > 让行半径 10.5 m；
            # 等 15 s 才退，退完停留 15 s）
            "unblock_back": 12.0,
            "unblock_max": 3,
            "unblock_wait": 15.0,
            "unblock_hold": 15.0,
            # 晚 2.5 s 再随机布点：A 已经放好之后，才能按 spawn_gap 避开它，
            # 否则两台车同一瞬间各自抽点，可能抽到贴在一起（实测遇到过）。
            "place_delay": 2.5,
            "vehicles": VEH_TOPICS,
            "pedestrians": "/pedestrians",
        }],
        # 抢位场景下第二台车改由 parker 恒定开进车位，不再跑环线
        condition=UnlessCondition(LaunchConfiguration("use_parker")),
    )

    # 行人：运动学驱动（调 gz 的 set_pose 服务沿**闭环**走），不依赖物理。
    # 路线按「区域合规」布设：人行道 / 斑马线 / 停车场，不上机动车道；
    # 每条都是闭环，开局在闭环上随机落点，遇车（半径 3.5 m）自动停步。
    # 行人需要调 gz 的 set_pose 服务，必须显式桥一个 ROS 服务出来
    # （config_file 只能桥话题，服务得写成命令行参数）
    set_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="set_pose_bridge",
        output="screen",
        arguments=["/world/city_parking_lot/set_pose"
                   "@ros_gz_interfaces/srv/SetEntityPose"
                   "@gz.msgs.Pose@gz.msgs.Boolean"],
    )
    PED_ROUTES = (
        # 1) 北侧人行道 + 东西两个路口斑马线（人行道 + 斑马线）
        #    [同学 C 修复] 南侧那段原来走 y=12.6 一直到 x=-5.5，正好停在停车场
        #    进出口那段转弯车道 `(-8,17)→(-3,14)→(-3,10)` 的旁边，
        #    净距只有 **2.49 m**（阈值 2.5）—— 也就是行人贴着进出车道走。
        #    改成：南侧人行道在 x=-8 处折下去到 y=11.0，并**跨过** x=-3 那条
        #    进出车道（过街，车会让人），这样既离开车道又保留「过斑马线」要素。
        "pedestrian_north=-21.8,26.0;-21.8,28.1;21.8,28.1;21.8,13.5;"
        "-8.0,12.6;-21.8,11.0"
        # 2) 停车场外环（停车场 + 周边人行道）
        "|pedestrian_lot_ring=-21.6,7.5;-21.6,-26.0;21.6,-26.0;21.6,7.5"
        # 3) 停车场内环：走**场心绿化带**那一圈，不碰任何车位的入位走廊。
        #    历程：最初 (-8,4.5)-(-8,-19)-(8,-19)-(8,4.5) 有三段正好压在干扰车
        #    的场内单向环上（净距 0.00 m），车必然从人身上开过去；于是外移到
        #    x=±11.5，虽然离干扰车环 3.5 m 了，但那是**过道本身**——以 W4 为例：
        #    车位中心 x=-15.5、车位口 x=-14.1，自车泊车前的预备位姿在
        #    (-12.5,-12.25)（车位中心沿轴外 1.6 m），车要从这里往回退 4.2 m 才进
        #    车位；而车身左沿在 x=-11.575，行人线 x=-11.5 只差 **0.15 m**。实测
        #    就是「行人站在车尾要退的线上」，泊车中途被卡死（check_ped_routes.py
        #    只看离车位**目标点**的距离，看不见预备位姿，所以当时还报了 OK
        #    ——已补上入位走廊检查）。
        #    现在贴着场心绿化带走：x=±3.0、y∈[-0.5,-15]，离场内车道（x=±8 与
        #    y=-19）都在 4.0 m 以上，离最近的入位走廊 4.18 m（P 排）；整条环完全
        #    落在停车场内（y≤-0.5），不碰场外进出车道（x=±3 那段在 y≥4），
        #    所以一次过街都不用。y=0.5/x=3.2 的版本曾被车道内切角压到 2.33 m。
        "|pedestrian_lot_inner=-3.0,-0.5;-3.0,-15.0;3.0,-15.0;3.0,-0.5"
        # 4) 停车场南环 + 南侧道路人行道（停车场 + 人行道）
        "|pedestrian_south=-21.6,-25.0;-21.6,-28.6;21.6,-28.6;21.6,-25.0"
        # 5) 东路口两个斑马线 + 北人行道（斑马线 + 人行道）
        #    [同学 C 修复] 原来走 `(21.8,13.5)→(29.0,12.6)→(36.2,13.0)` 那条南侧
        #    人行道，它有两个问题：东街车道 x=27 在 y=12 处就在旁边（净距 **1.74 m**，
        #    人车同道），而且整段压在 SE5/SE4 车位口上方（离 SE5 只有 1.57 m）。
        #    南侧那条带子本身只有 1 m 宽（主路南沿 y≈13，SE 车位顶边 y=12），
        #    怎么摆都在车道里，所以干脆不走它：改成沿停车场东侧人行带（x=21.6）
        #    下到南侧，再从 x=36.2 的斑马线上去 —— 两条竖线都是已经验证过的
        #    安全带（`pedestrian_lot_ring` 用的就是 x=±21.6）。
        "|pedestrian_east=21.8,26.0;21.8,11.8;21.6,-26.0;36.2,-28.6;"
        "36.2,26.0;29.0,28.1"
    )
    peds = Node(
        package="scene_agents",
        executable="pedestrian_agent",
        name="pedestrian_agent",
        output="screen",
        parameters=[{
            "world": "city_parking_lot",
            "speed": 1.1,
            "rate": 10.0,
            # 路权：行人优先，车让行。行人不设「让行超时强行通过」，
            # 只在车压到 evade_clear 以内时让行（evade_max 米预算），
            # 贴住（<0.35 m）就站住，绝不向前挤。
            # [同学 C] evade_clear 1.6 → 3.0，并新增「侧向让开 2.8 m」：
            # 干扰车的让行距离是 ped_stop=7.0 m，人这边只有 1.6 m 时会掉进
            # 「车觉得被堵、人觉得自己没挡路」的空档，两边都不动（截图里的死锁）。
            # 人的触发距离必须大于车的，而且只沿自己的路后退没用（后退方向常常
            # 与车的走廊平行），所以还要横向挪出 2.8 m（> 车走廊半宽 2.4 m）。
            "evade_clear": 3.0,
            "side_step": 2.8,
            "side_speed": 0.6,
            "evade_max": 15.0,
            "resume_radius": 4.5,
            # 让行扇形全角（度）：<180°，只对「前进方向上」的车辆让行。
            # 180° 等于旧的全向圆判据（车在侧面/后方也停）。
            "fov": 140.0,
            # 被挡后的动作序列：先等 2.5 s → 仍被挡就沿原路径退 15 m →
            # 再站住 10.5 s（≥10 s，给车足够时间通过）→ 然后才重新前进。
            # 替代原来「一格格往前蹭」的蠕动行为。
            # back_dist 2.0 → 15.0（2026-10-28）：2 m 的退让量出不了车的走廊
            # （走廊半宽就有 2.4 m），车还得继续等；一次退 15 m 直接离开整段
            # 车道。注意 evade_max=15.0 的预算恰好覆盖一次全程后退，用尽后
            # 靠「前进 3 m / 挡路车静止 12 s」复位，不会把人永久冻在原地。
            "wait_before_back": 2.5,
            "back_dist": 15.0,
            "back_speed": 0.6,
            "hold_after_back": 10.5,
            "vehicles": VEH_TOPICS,
            "routes": PED_ROUTES,
        }],
    )
    parker = Node(
        package="scene_agents",
        executable="parker",
        name="parker",
        output="screen",
        parameters=[{
            "odom_topic": "/traffic2/odom_gt",
            "cmd_topic": "/traffic2/cmd_vel",
            "speed": 1.8,
            "loop": False,
        }],
        condition=IfCondition(LaunchConfiguration("use_parker")),
    )

    # ---------------- 同学 B：检测 → 预测 → 风险 ----------------
    detection = Node(
        package="detection_tracking",
        executable="detection_node",
        name="detection_node",
        output="screen",
        condition=IfCondition(use_prediction),
    )
    prediction = Node(
        package="trajectory_prediction",
        executable="predictor_node",
        name="predictor_node",
        output="screen",
        parameters=[{"model_path": model_path, "pose_topic": "/vehicle_pose"}],
        condition=IfCondition(use_prediction),
    )
    risk = Node(
        package="risk_assessment",
        executable="risk_node",
        name="risk_node",
        output="screen",
        parameters=[{
            "pose_topic": "/vehicle_pose",
            # map 帧原点即自车生成位姿，兜底值只用于 /target_spot 到达前的极短时间
            "target_spot_x": 0.0,
            "target_spot_y": 0.0,
            "corridor_half_width": 1.2,
        }],
        condition=IfCondition(use_prediction),
    )

    # ---------------- 同学 C：泊车规划控制 ----------------
    planner = Node(
        package="parking_planner",
        executable="planner_node",
        name="parking_planner",
        output="screen",
        parameters=[{
            "layout_path": os.path.join(planner_pkg, "config", "lot_layout.yaml"),
            "goal_stall": ParameterValue(goal_stall, value_type=str),
            "stall_region": ParameterValue(stall_region, value_type=str),
            "prefer_stalls": ParameterValue(prefer_stalls, value_type=str),
            "max_speed": ParameterValue(max_speed, value_type=float),
            "eval_csv": ParameterValue(eval_csv, value_type=str),
            "spawn_x": ParameterValue(spawn_x, value_type=float),
            "spawn_y": ParameterValue(spawn_y, value_type=float),
            "spawn_yaw": ParameterValue(spawn_yaw, value_type=float),
            "route_speed": 1.4,
            "warmup_scans": 12,
            "use_sim_time": True,
        }],
    )
    mux = Node(
        package="parking_planner",
        executable="cmd_mux",
        name="cmd_mux",
        output="screen",
        parameters=[{"use_sim_time": True}],
    )
    evaluator = Node(
        package="parking_planner",
        executable="parking_eval",
        name="parking_eval",
        output="screen",
        parameters=[{
            "csv": os.path.expanduser("~/parking_trace.csv"),
            "use_sim_time": True,
        }],
    )
    # [同学 C] 每次演示顺带记一份 SLAM 定位精度（ATE），否则「用了 slam_toolbox」
    # 这句话没有数字支撑。写成 ~/apa_ws/logs/ate.csv + ate_summary.txt。
    slam_eval = Node(
        package="parking_slam",
        executable="slam_eval",
        name="slam_eval",
        output="screen",
        parameters=[{
            "csv": os.path.expanduser("~/apa_ws/logs/ate.csv"),
            "use_sim_time": True,
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("headless", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("drive_patrol", default_value="false"),
        DeclareLaunchArgument("use_prediction", default_value="true"),
        DeclareLaunchArgument("use_parker", default_value="false"),
        # [同学 C 新增] 主路循环干扰车（traffic_car / traffic_car_b）开关。
        # 默认 true —— 与 GUI 里看到的场景完全一致。批量逐车位回归时置 false：
        # 两辆干扰车与自车在主干道上会**对向互相让行**（干扰车「前方 9 m 内有车
        # 停车让行」，自车激光护栏「走廊内有障碍，停止」），实测把一轮 W7 卡在
        # 场外 262 s、路线超时后重新选位，逐车位数据就不可比了。
        DeclareLaunchArgument("use_traffic", default_value="true"),
        # [同学 C 新增] 行人开关，默认 true（与 GUI 一致）。
        # 逐车位回归时置 false：5 组行人会横穿进场车道，自车「巡航让行」一直等到
        # 路线超时（实测 W7 一轮卡在 1793010048 报「全局路线行驶超时，放弃」）。
        # 行人本身的让行/绕行逻辑另有 `selftest_ped_*.py` + `check_ped_routes.py` 覆盖。
        DeclareLaunchArgument("use_peds", default_value="true"),
        DeclareLaunchArgument("goal_stall", default_value=""),
        DeclareLaunchArgument("stall_region", default_value=""),
        DeclareLaunchArgument("prefer_stalls", default_value=""),
        DeclareLaunchArgument("max_speed", default_value="0.45"),
        DeclareLaunchArgument("spawn_x", default_value="-45.0"),
        DeclareLaunchArgument("spawn_y", default_value="15.65"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "eval_csv", default_value=os.path.expanduser("~/parking_eval.csv")),
        DeclareLaunchArgument(
            "model_path",
            default_value=os.path.expanduser(
                "~/apa_ws/training/checkpoints/trajectory_transformer.pt")),
        city,
        TimerAction(period=9.0, actions=[traffic],
                    condition=IfCondition(LaunchConfiguration("use_traffic"))),
        TimerAction(period=9.5, actions=[traffic_b],
                    condition=IfCondition(LaunchConfiguration("use_traffic"))),
        TimerAction(period=10.0, actions=[set_pose_bridge]),
        TimerAction(period=11.0, actions=[peds],
                    condition=IfCondition(LaunchConfiguration("use_peds"))),
        TimerAction(period=13.0, actions=[mux]),
        TimerAction(period=16.0, actions=[planner]),
        TimerAction(period=19.0, actions=[evaluator]),
        TimerAction(period=9.0, actions=[slam_eval]),
        TimerAction(period=12.0, actions=[detection]),
        TimerAction(period=15.0, actions=[prediction]),
        TimerAction(period=18.0, actions=[risk]),
        TimerAction(period=14.0, actions=[parker]),
    ])
