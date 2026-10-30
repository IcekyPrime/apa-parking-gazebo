#!/usr/bin/env python3
"""生成“城市一角”停车场 Gazebo 场景：双行道 + 两侧十字路口 + 中央露天停车场。

布局（x 向东，y 向北，单位 m）：
  主路（双行道，每向 2 车道）  y ∈ [14, 26]
  北侧人行道                  y ∈ [26, 30]
  停车场                      x ∈ [-20, 20], y ∈ [-24, 6]
  停车场入口连接段            x ∈ [-5, 5],  y ∈ [6, 14]
  西侧街 x ∈ [-33, -25] / 东侧街 x ∈ [25, 33]，各与主路形成十字路口
  南侧道路                    y ∈ [-38, -30]

[同学 C 改造记录]
1. 自车不再写进 world，改为 launch 时用 `ros_gz_sim create` 按参数生成，
   因此主路上任意位置都可以作为演示起点（见 scripts/run_demo.sh 的 SPAWN_*）；
2. 西侧街、东侧街新增路边平行车位（各 6 个），加上南侧道路原有的 6 个，
   `任意空车位` 才真正做到覆盖四条道路；
3. 行人加上 collision（原来只有 visual，激光雷达看不到），并新增一名在停车场
   内沿过道行走的行人，用于测试泊车过程中的动态避让；
4. 新增 `traffic_car_b`（干扰车 B），用于“匀速行驶并停入某个车位”的测试场景。
"""
import argparse
import math
import os

ROAD = (0.22, 0.22, 0.24)
LOT = (0.27, 0.27, 0.29)
SIDEWALK = (0.72, 0.72, 0.70)
CURB = (0.62, 0.62, 0.60)
GRASS = (0.24, 0.44, 0.18)
WHITE = (0.92, 0.92, 0.90)
YELLOW = (0.86, 0.74, 0.16)

# 西/东侧街路边车位：沿 y 方向每 6 m 一个，深 2.4 m
SIDE_STALL_Y = (-21.0, -15.0, -9.0, -3.0, 3.0, 9.0)
SIDE_STALL_DEPTH = 2.4
SIDE_STALL_LEN = 6.0

# [本轮新增] 场内南侧 P 排平行车位 + 中央岛的**可调几何**（默认值与旧版完全一致）：
#   ISLAND_SOUTH : 中央绿化岛南沿 y（默认 -16）。它与 P 排车位口（-21.6）之间
#                  就是 P 排的过道（默认 5.6 m）。把岛往北收（如 -13）等于
#                  给平行泊车留出更多摆正空间。
#   P_DEPTH      : P 排车位深度（默认 2.4 m）。车位口固定在 y=-21.6，
#                  加深 = 车位往南延伸（需要同步延长 lot_surface 与南环人行道）。
#   P_PITCH      : P 排车位长度/间距（默认 6.0 m）。排的西端锚定在 x=-15 不动，
#                  加大间距 = 车位向东延伸（受东侧垂直车位口 x=14.1 限制）。
ISLAND_SOUTH = -16.0
P_DEPTH = 2.4
P_PITCH = 6.0
# [本轮新增] B 排（南侧道路路边平行位）车位长度/间距。7 条分隔线以 x=0 居中。
# 6.0 → 6.6 → 7.2：B3 两侧（B2/B4）都停着车，倒车弧会扫到邻车的东端。
# 单向排布 + 6.6 时离线余量只有 +0.075 m（几乎贴住 parked_b2）；
# 改为居中排布后升到 +0.77，间距再加到 7.2 后为 +0.83、终点误差 0.11 m/3.1°。
# 上限：居中后车位两端在 x=±21.6，西/东侧街路口自 ±24.4 起；
# 若取 7.8 则车位端到 ±23.4，而 B5 的预备位（车位中心东移 6.1 m）会落到
# 路口里（x=25.6），所以停在 7.2。
B_PITCH = 7.2

parts = []


def material(color, spec=0.1):
    r, g, b = color
    return (
        f"<material><ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>"
        f"<diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>"
        f"<specular>{spec:.3f} {spec:.3f} {spec:.3f} 1</specular></material>"
    )


def box(name, cx, cy, cz, sx, sy, sz, color, yaw=0.0, collide=True):
    c = ""
    if collide:
        c = f'<collision name="collision"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry></collision>'
    parts.append(
        f'    <model name="{name}">\n'
        f"      <static>true</static>\n"
        f"      <pose>{cx} {cy} {cz} 0 0 {yaw}</pose>\n"
        f'      <link name="link">\n'
        f"        {c}\n"
        f'        <visual name="visual"><geometry><box><size>{sx} {sy} {sz}</size></box></geometry>{material(color)}</visual>\n'
        f"      </link>\n"
        f"    </model>"
    )


def cylinder(name, cx, cy, cz, r, h, color, collide=True):
    c = ""
    if collide:
        c = f'<collision name="collision"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry></collision>'
    parts.append(
        f'    <model name="{name}">\n'
        f"      <static>true</static>\n"
        f"      <pose>{cx} {cy} {cz} 0 0 0</pose>\n"
        f'      <link name="link">\n'
        f"        {c}\n"
        f'        <visual name="visual"><geometry><cylinder><radius>{r}</radius><length>{h}</length></cylinder></geometry>{material(color)}</visual>\n'
        f"      </link>\n"
        f"    </model>"
    )


def include(name, uri, x, y, z=0.0, yaw=0.0):
    parts.append(
        f"    <include><uri>model://{uri}</uri><name>{name}</name>"
        f"<pose>{x} {y} {z} 0 0 {yaw}</pose></include>"
    )


def crossings_and_lanes():
    """车道线、双黄线、斑马线、停止线、各类车位线"""
    n = 0
    # 主路边缘实线
    for y in (14.3, 25.7):
        n += 1
        box(f"edge_line_{n}", 0, y, 0.055, 190, 0.14, 0.01, WHITE, collide=False)
    # 中央双黄线（路口处断开，两端止于停止线）
    for x0, x1 in ((-95.0, -39.0), (-19.0, 19.0), (39.0, 95.0)):
        for y in (19.86, 20.14):
            n += 1
            box(f"center_line_{n}", (x0 + x1) / 2, y, 0.055, x1 - x0, 0.12, 0.01,
                YELLOW, collide=False)
    # 车道虚线
    for lane_y in (17.0, 23.0):
        x = -93.0
        while x < 93.0:
            if not (-39.0 <= x <= -19.0 or 19.0 <= x <= 39.0):
                n += 1
                box(f"dash_{n}", x, lane_y, 0.055, 3.0, 0.12, 0.01, WHITE, collide=False)
            x += 6.0
    # 斑马线（两个十字路口西/东两侧横穿主路）
    for cx in (-36.2, 36.2):
        y = 14.7
        while y < 25.5:
            n += 1
            box(f"zebra_{n}", cx, y, 0.055, 3.4, 0.55, 0.01, WHITE, collide=False)
            y += 1.15
    # 斑马线（路口靠内侧）
    for cx in (-21.8, 21.8):
        y = 14.7
        while y < 25.5:
            n += 1
            box(f"zebra_in_{n}", cx, y, 0.055, 3.4, 0.55, 0.01, WHITE, collide=False)
            y += 1.15
    # 斑马线（南北向道路的路口南北两侧）
    for road_x, cy in ((-29.0, 28.1), (-29.0, 12.6), (29.0, 28.1), (29.0, 12.6)):
        x = road_x - 3.6
        while x < road_x + 3.6:
            n += 1
            box(f"zebra_ns_{n}", x, cy, 0.055, 0.55, 3.4, 0.01, WHITE, collide=False)
            x += 1.15
    # 停止线
    for cx, y0 in ((-39.0, 17.0), (-19.0, 23.0), (19.0, 17.0), (39.0, 23.0)):
        n += 1
        box(f"stop_line_{n}", cx, y0, 0.055, 0.36, 5.9, 0.01, WHITE, collide=False)
    # 停车场车位线（2.5 m 一档）
    stall_y0, stall_pitch, stall_n = -23.5, 2.5, 10
    stall_w = 19.9 - 14.1
    for k in range(stall_n + 1):
        yb = stall_y0 + k * stall_pitch
        for cx in (-17.0, 17.0):
            n += 1
            box(f"stall_line_{n}", cx, yb, 0.055, stall_w, 0.12, 0.01, WHITE, collide=False)
    # 侧边车位边界线
    stall_len = stall_n * stall_pitch
    for cx in (-19.9, -14.1, 14.1, 19.9):
        n += 1
        box(f"stall_edge_{n}", cx, stall_y0 + stall_len / 2, 0.055, 0.12, stall_len,
            0.01, WHITE, collide=False)
    # 平行车位（场内南侧通道边）
    # 整排比停车场中线西移 3 m：平行泊车要「先开过车位再倒车入位」，车位东侧
    # 必须留出至少一个车身（4.4 m）的驶过余量。原来从 x=-12 起排，最东那个车位
    # 中心在 x=9，驶过点 14.2 m 已经顶到东侧垂直车位的车位口（x=14.1），
    # 车辆在预备位姿就会和旁边停着的车重叠。
    # [本轮] 车位口固定 y=-21.6，深度/间距可调（P_DEPTH / P_PITCH，见文件头）。
    para_y = -21.6 - P_DEPTH / 2.0
    for i in range(5):
        n += 1
        box(f"para_line_{n}", -15.0 + P_PITCH * i, para_y, 0.055, 0.12, P_DEPTH,
            0.01, WHITE, collide=False)
    n += 1
    box("para_edge", -15.0 + 2.0 * P_PITCH, -21.6, 0.055, 4.0 * P_PITCH, 0.12,
        0.01, WHITE, collide=False)
    # 停车场后方道路（y=-34）靠停车场一侧的一排平行车位
    # [本轮修复] B 排车位整体南移 0.8 m（-31.2 → -32.0），给倒车弧留缓冲。
    # 原来车位北沿正好压在 y=-30.0，而 `sidewalk_south_road_*` 的南沿也在
    # -30.0 —— 平行泊车的 RS 解必然把车身北角甩出去 0.4~0.5 m，
    # 离线量到**余量 0.00 m**、真机 B3 执行后间隙 0.01 m（压上人行道）。
    # 车位南移后北沿 -30.8，留 0.8 m 缓冲；南侧道路同步加宽 1 m（8→9，
    # 中心 -34 → -34.5），保证车位南侧的行车道仍有 5.8 m。
    # [本轮修复] B 排由「自 x=-18 向东单向排布」改为**以 x=0 为中心对称排布**。
    # 原因：南侧道路两端 x=±24.4 处是西/东侧街的路口，单向排布时西端空 6.4 m、
    # 东端只剩 2.8 m；而 B3 的倒车弧会扫到 B4 的停放车（离线余量仅 +0.075 m），
    # 需要把间距从 6.6 加大到 7.8 —— 单向排布下 B5 会被推进东侧路口
    # （车位占 x∈[21.0,28.8]，路口自 24.4 起）。居中后两端各余 1.0 m。
    for i in range(7):
        n += 1
        box(f"back_stall_{n}", -3.0 * B_PITCH + B_PITCH * i, -32.0, 0.055,
            0.12, 2.4, 0.01, WHITE, collide=False)
    n += 1
    box(f"back_stall_edge_{n}", 0, -33.2, 0.055, 6.0 * B_PITCH, 0.12, 0.01, WHITE,
        collide=False)

    # ---------------- [同学 C 新增] 西/东侧街路边平行车位 ----------------
    # 车道宽 8 m，靠外侧（西街靠西、东街靠东）划 2.4 m 深的车位，6 m 一档
    for side, road_cx, out_dir in (("W", -29.0, -1.0), ("E", 29.0, +1.0)):
        edge_x = road_cx + out_dir * (4.0 - 0.15)          # 车位外边界（靠街边）
        inner_x = edge_x - out_dir * SIDE_STALL_DEPTH      # 车位内边界（靠车道）
        cx = 0.5 * (edge_x + inner_x)
        for i, y0 in enumerate(SIDE_STALL_Y):
            # 每档的分隔线（沿 x 方向，长度 = 车位深）
            n += 1
            box(f"side_stall_{side}_{i}", cx, y0 - SIDE_STALL_LEN / 2, 0.055,
                SIDE_STALL_DEPTH, 0.12, 0.01, WHITE, collide=False)
        # 末尾封口线
        n += 1
        box(f"side_stall_{side}_end", cx, SIDE_STALL_Y[-1] + SIDE_STALL_LEN / 2,
            0.055, SIDE_STALL_DEPTH, 0.12, 0.01, WHITE, collide=False)
        # 车位内边界实线
        n += 1
        box(f"side_stall_{side}_edge", inner_x, 0.5 * (SIDE_STALL_Y[0] - SIDE_STALL_LEN / 2 +
                                                       SIDE_STALL_Y[-1] + SIDE_STALL_LEN / 2),
            0.055, 0.12, SIDE_STALL_Y[-1] - SIDE_STALL_Y[0] + SIDE_STALL_LEN,
            0.01, WHITE, collide=False)


def pedestrian(name, color, pose0, yaw0):
    """行人模型：静态 + 碰撞体，运动由 `scene_agents/pedestrian_agent` 驱动。

    两个必须保留的点：
    1) **要有 collision**。`<static>` 模型只要没有 collision，gpu_lidar 就完全扫不到，
       行人等于对自车不存在，动态避障测试也就没有意义。
    2) **保持 `<static>true</static>`**。行人的运动由 `pedestrian_agent` 调 gz 的
       `/world/<world>/set_pose` 服务搬运（运动学，不走物理）。以前用 Gazebo 自带的
       TrajectoryFollower（靠施加推力驱赶），它不会提前减速，实测冲过航点 4~16 m，
       而且圆柱碰撞体没有轮子，一碰到人行道那种 0.12 m 台阶就永久卡死 ——
       表现就是「画面里行人一动不动」。静态模型不受重力、也不会被车推走，
       加上服务驱动后行为完全可预测。

    行走路线写在 launch 的 `routes` 参数里（见 parking_demo.launch.py），
    因为那是「行为」而不是「场景几何」。
    """
    r, g, b = color
    return f"""    <model name="{name}">
      <static>true</static>
      <pose>{pose0[0]} {pose0[1]} 0.05 0 0 {yaw0}</pose>
      <link name="body">
        <collision name="c"><pose>0 0 0.78 0 0 0</pose><geometry><cylinder><radius>0.24</radius><length>1.5</length></cylinder></geometry></collision>
        <visual name="torso"><pose>0 0 0.90 0 0 0</pose><geometry><cylinder><radius>0.19</radius><length>1.0</length></cylinder></geometry><material><ambient>{r:.2f} {g:.2f} {b:.2f} 1</ambient><diffuse>{r:.2f} {g:.2f} {b:.2f} 1</diffuse></material></visual>
        <visual name="head"><pose>0 0 1.55 0 0 0</pose><geometry><sphere><radius>0.15</radius></sphere></geometry><material><ambient>0.82 0.66 0.52 1</ambient><diffuse>0.85 0.68 0.54 1</diffuse></material></visual>
      </link>
    </model>"""


def build():
    head = """<?xml version="1.0" ?>
<!-- 城市一角：停车场 + 双行道 + 两侧十字路口（由 tools/build_world.py 生成） -->
<sdf version="1.9">
  <world name="city_parking_lot">
    <physics name="physics" type="ignored">
      <max_step_size>0.002</max_step_size>
      <real_time_factor>1.0</real_time_factor>
      <real_time_update_rate>500</real_time_update_rate>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <scene>
      <ambient>0.55 0.55 0.58 1</ambient>
      <background>0.72 0.80 0.90 1</background>
      <shadows>false</shadows>
    </scene>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 30 0 0 0</pose>
      <direction>-0.45 0.35 -1.0</direction>
      <diffuse>0.95 0.95 0.92 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
    </light>
"""
    parts.append(
        '    <model name="ground">\n'
        "      <static>true</static>\n"
        '      <link name="link">\n'
        '        <collision name="collision"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry></collision>\n'
        f'        <visual name="visual"><geometry><plane><normal>0 0 1</normal><size>400 400</size></plane></geometry>{material((0.30, 0.34, 0.28))}</visual>\n'
        "      </link>\n"
        "    </model>"
    )

    # ---------------- 路面 ----------------
    box("road_main", 0, 20, 0.02, 190, 12, 0.04, ROAD)
    box("road_driveway", 0, 10, 0.02, 10, 8, 0.04, ROAD)
    box("road_west", -29, 1, 0.02, 8, 78, 0.04, ROAD)
    box("road_east", 29, 1, 0.02, 8, 78, 0.04, ROAD)
    # [本轮] 南侧道路加宽 1 m 并南移 0.5 m：B 排车位南移后要保持行车道宽度
    box("road_south", 0, -34.5, 0.02, 120, 9, 0.04, ROAD)
    # [本轮] 铺面南沿跟随 P 排深度延伸（车位口 -21.6 - 深度），保证车位底不落在草皮上
    lot_south = min(-24.0, -21.6 - P_DEPTH - 0.4)
    box("lot_surface", 0, (6.0 + lot_south) / 2.0, 0.02, 40, 6.0 - lot_south, 0.04,
        LOT)

    # ---------------- 人行道 / 路缘 ----------------
    for i, (x0, x1) in enumerate(((-95, -34), (-24, 24), (34, 95))):
        box(f"sidewalk_north_{i}", (x0 + x1) / 2, 28.1, 0.06, x1 - x0, 4.2, 0.12, SIDEWALK)
        box(f"grass_north_{i}", (x0 + x1) / 2, 31.4, 0.05, x1 - x0, 2.8, 0.10, GRASS)
    # [本轮修复] 南侧道路人行道中段两端各退 2.5 m（-24→-21.5、24→21.5）。
    # 原因：西街(x=-28.4, 宽8) 与南侧道路(y=-34.5, 宽9) 的路口，做 90° 转弯
    # 需要 R≈6 m 的切线，弧的北端点落在 y=-28.46 —— 正好压在原人行道
    # （y∈[-30,-28.4]）上。离线实测 B0~B5 的路线在 x∈[-28.3,-23.3] 这一段
    # 的曲率 R=4.01（低于车辆极限 5.74），车身必然切角；切角之后激光护栏
    # 就扫到这块 0.12 m 高的路缘 → 真机表现为「路线前方有障碍，停车」卡死。
    # 真实路口的做法就是把人行道在转角处退让（加铺面缺口/降坡），
    # 这里给人行道两端各让 2.5 m（东侧同理：es_m34 的弧端点 x=22.36）。
    for i, (x0, x1) in enumerate(((-60, -34), (-21.5, 21.5), (34, 60))):
        box(f"sidewalk_south_road_{i}", (x0 + x1) / 2, -29.2, 0.06, x1 - x0, 1.6, 0.12, SIDEWALK)
    for i, (x0, x1) in enumerate(((-95, -34), (-24, -5.5), (5.5, 24), (34, 95))):
        box(f"sidewalk_south_{i}", (x0 + x1) / 2, 12.6, 0.06, x1 - x0, 2.8, 0.12, SIDEWALK)
    box("lot_ring_w", -21.6, -9, 0.06, 3.2, 30, 0.12, SIDEWALK)
    box("lot_ring_e", 21.6, -9, 0.06, 3.2, 30, 0.12, SIDEWALK)
    # [本轮] 南环人行道与车位背之间留 1.0 m 缓冲带：
    # 平行泊车的 RS 解析解倒车弧会超出车位背 0.4~0.5 m（真机离线量到
    # 压上旧人行道、余量 0.05 / 执行剐蹭 0.00），铺面多出 0.4 m 不够，
    # 必须再往外让 1.0 m。人行道本身收窄到 2.0 m（原来 3.2）。
    box("lot_ring_s", 0, lot_south - 2.0, 0.06, 46, 2.0, 0.12, SIDEWALK)
    box("lot_ring_n_w", -14.5, 7.6, 0.06, 17, 3.2, 0.12, SIDEWALK)
    box("lot_ring_n_e", 14.5, 7.6, 0.06, 17, 3.2, 0.12, SIDEWALK)

    # ---------------- 绿化带 ----------------
    box("grass_lot_w", -22.8, -9, 0.05, 1.4, 28, 0.10, GRASS)
    box("grass_lot_e", 22.8, -9, 0.05, 1.4, 28, 0.10, GRASS)

    crossings_and_lanes()

    # ---------------- 中央绿化隔离带 + 立柱 ----------------
    # [本轮] 南沿可调（ISLAND_SOUTH）：北沿固定在 y=-4，向南收缩等于给 P 排过道让位
    island_h = -4.0 - ISLAND_SOUTH
    island_cy = (ISLAND_SOUTH - 4.0) / 2.0
    box("island_curb", 0, island_cy, 0.09, 3.2, island_h, 0.18, CURB)
    box("island_grass", 0, island_cy, 0.19, 2.8, island_h - 0.4, 0.10, GRASS)
    for i, (px, py) in enumerate(((-1.0, -5.5), (1.0, -5.5),
                                  (-1.0, ISLAND_SOUTH + 1.5),
                                  (1.0, ISLAND_SOUTH + 1.5))):
        cylinder(f"pillar_i{i}", px, py, 1.6, 0.26, 3.2, (0.55, 0.55, 0.53))
    for i, (px, py) in enumerate(((-7.0, 5.5), (7.0, 5.5))):
        cylinder(f"pillar_e{i}", px, py, 1.6, 0.26, 3.2, (0.55, 0.55, 0.53))

    # ---------------- 建筑 ----------------
    buildings = [
        ("bldg_nw", -55, 44.5, 24, 18, 22, (0.76, 0.74, 0.70)),
        ("bldg_n", -6, 46, 26, 18, 30, (0.60, 0.66, 0.74)),
        ("bldg_ne", 44, 44.5, 24, 18, 18, (0.78, 0.68, 0.58)),
        ("bldg_w", -56, -6, 26, 24, 14, (0.70, 0.72, 0.68)),
        ("bldg_e", 56, -6, 26, 24, 16, (0.74, 0.70, 0.66)),
        ("bldg_s", 0, -56, 32, 20, 12, (0.66, 0.70, 0.72)),
    ]
    for name, cx, cy, sx, sy, h, color in buildings:
        box(name, cx, cy, h / 2, sx, sy, h, color)
        box(name + "_roof", cx + sx * 0.18, cy, h + 1.0, sx * 0.28, sy * 0.4, 2.0, (0.5, 0.5, 0.5), collide=False)
        floors = max(2, int(h / 4))
        for f in range(1, floors):
            box(
                f"{name}_win{f}",
                cx,
                cy,
                f * h / floors,
                sx + 0.12,
                sy + 0.12,
                1.1,
                (0.24, 0.30, 0.36),
                collide=False,
            )

    # ---------------- 树 ----------------
    tree_pos = [(x, 31.4) for x in range(-88, 89, 11)]
    tree_pos += [(-22.6, y) for y in (-22, -14, -6, 2)]
    tree_pos += [(22.6, y) for y in (-22, -14, -6, 2)]
    tree_pos += [(-33.6, y) for y in (-24, -10, 4)]
    tree_pos += [(33.6, y) for y in (-24, -10, 4)]
    for i, (tx, ty) in enumerate(tree_pos):
        include(f"tree_{i}", "tree", tx, ty)

    # ---------------- 车位里的车 ----------------
    occupied_west = [0, 2, 3, 6, 8]
    occupied_east = [1, 4, 5, 9]
    for k in occupied_west:
        include(f"parked_w{k}", "parked_car", -17.0, -23.5 + 1.25 + 2.5 * k, 0.02, 0.0)
    for k in occupied_east:
        include(f"parked_e{k}", "parked_car", 17.0, -23.5 + 1.25 + 2.5 * k, 0.02, math.pi)
    # [本轮] P 排停驻车跟随 P_PITCH / P_DEPTH 定位（P0 与 P2 有车，P1/P3 空）
    for i, cx in enumerate((-15.0 + P_PITCH / 2.0, -15.0 + 2.5 * P_PITCH)):
        include(f"parked_p{i}", "parked_car", cx, -21.6 - P_DEPTH / 2.0, 0.02, 0.0)
    # [本轮] B 排停驻车跟随车位南移（-31.2 → -32.0）、间距（B_PITCH）与居中排布
    for i, k in enumerate((0, 2, 4)):
        include(f"parked_b{i}", "parked_car",
                -3.0 * B_PITCH + B_PITCH * (k + 0.5), -32.0, 0.02, 0.0)
    # [同学 C 新增] 西/东侧街路边车位各停 2 辆，留出空位供泊车
    # 注意两点：
    #  * 侧街车位长轴沿 y，停进去的车必须 yaw=±pi/2，否则车身横在车道上
    #    （第一版踩过这个坑）；
    #  * 朝向取 -pi/2（车头朝南），与 lot_layout 里车位的标定一致 —— 车从主路
    #    右转进入侧街是自北向南，平行泊车要「开过车位再倒车入位」，停好车头朝南。
    for side, px in (("W", -31.65), ("E", 31.65)):
        for i in (0, 3):
            include(f"parked_side_{side}{i}", "parked_car", px,
                    SIDE_STALL_Y[i], 0.02, -math.pi / 2)

    # ---------------- 动态交通车（社会车辆） ----------------
    # 两辆干扰车都**写进 world**（静态加载），界面首帧就能看到；
    # 行驶路线 / 随机起点 / 遇车停车由 scene_agents 的 traffic_driver(A/B) 驱动。
    # 以前只放一台、第二台靠 use_parker 运行期 create，结果是：不开抢位场景时
    # 没有任何节点驱动它，会永久停在西侧路口中间，看起来像故障。
    include("traffic_car", "city_car", -60.0, 15.65, 0.03, 0.0)
    include("traffic_car_b", "city_car_b", 60.0, 23.0, 0.03, math.pi)

    # ---------------- 行人 ----------------
    # 模型只负责「站在哪」（初始位姿）；行走路线在 launch 的 routes 参数里，
    # 由 scene_agents/pedestrian_agent 通过 gz 的 set_pose 服务搬运。
    # 区域合规：行人只走人行道 / 斑马线 / 停车场，不上机动车道。
    # 每组一条**闭环**路径，开局在闭环上随机落点（见 pedestrian_agent）。
    parts.append(pedestrian("pedestrian_north", (0.15, 0.25, 0.62),
                            (-8.0, 28.1), 1.5707))
    parts.append(pedestrian("pedestrian_lot_ring", (0.62, 0.22, 0.20),
                            (-21.6, -10.0), 4.7124))
    parts.append(pedestrian("pedestrian_lot_inner", (0.20, 0.50, 0.25),
                            (-8.0, -8.0), 4.7124))
    parts.append(pedestrian("pedestrian_south", (0.58, 0.42, 0.10),
                            (0.0, -28.6), 0.0))
    parts.append(pedestrian("pedestrian_east", (0.42, 0.20, 0.58),
                            (36.2, 20.0), 3.1416))

    # 注意：自车（ego_vehicle）不再写入 world —— 由 launch 用 ros_gz_sim create
    # 按 SPAWN_X / SPAWN_Y / SPAWN_YAW 参数生成，因此主路上任意位置都能作为起点。

    return head + "\n".join(parts) + "\n  </world>\n</sdf>\n"


if __name__ == "__main__":
    ap = argparse.ArgumentParser(description='生成 city_parking_lot 场景')
    ap.add_argument('--island-south', type=float, default=ISLAND_SOUTH,
                    help='中央岛南沿 y（默认 -16，最大收到 -13）')
    ap.add_argument('--p-depth', type=float, default=P_DEPTH,
                    help='P 排平行车位深度（默认 2.4，上限 5.4）')
    ap.add_argument('--p-pitch', type=float, default=P_PITCH,
                    help='P 排平行车位长度/间距（默认 6.0，上限 6.15）')
    ap.add_argument('--b-pitch', type=float, default=B_PITCH,
                    help='B 排平行车位长度/间距（默认 6.6）')
    ns = ap.parse_args()
    ISLAND_SOUTH = ns.island_south
    P_DEPTH = ns.p_depth
    P_PITCH = ns.p_pitch
    B_PITCH = ns.b_pitch
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.makedirs(os.path.join(root, "worlds"), exist_ok=True)
    out = os.path.join(root, "worlds", "city_parking_lot.sdf")
    with open(out, "w", encoding="utf-8") as f:
        f.write(build())
    print("wrote", out, len(build().splitlines()), "lines",
          f'(island_south={ISLAND_SOUTH}, p_depth={P_DEPTH}, p_pitch={P_PITCH})')
