#!/usr/bin/env python3
"""检验 E2 锚定解路径的内部一致性：yaw 字段 vs 位置折线切线。

背景：真机日志显示 φ≈+2° 却有 de/ds≈-0.30 的横向误差增长率 —— 运动学上
不可能，除非路径的 yaw 字段相对位置折线存在 ~17° 的系统性偏角
（κ×参数错位 ≈ 0.165×2 m ≈ 19°）。逐点对比两者。
"""
import math
import os
import sys

WS = os.path.expanduser('~/apa_ws')
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
from parking_planner.geometry import wrap
from parking_planner.hybrid_astar import HybridAStar
from parking_planner.lot_layout import Layout, WHEELBASE
from parking_planner.route_planner import RouteGraph, default_graph_path

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds', 'city_parking_lot.sdf')
SPAWN = (-30.0, 15.65, 0.0)

layout = Layout.from_world(WORLD).to_frame(SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
st = layout.find_stall('E2')
route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
hand = route[-1]
pts = [hand, st.goal_pose, st.staging_pose]
cx = sum(p[0] for p in pts) / 3.0
cy = sum(p[1] for p in pts) / 3.0
half = max(15.0, max(math.hypot(p[0] - cx, p[1] - cy) for p in pts) + 6.0)
grid = layout.occupancy_grid(0.125, roi=(cx - half, cx + half,
                                         cy - half, cy + half))
for ms in (0.42, 0.44):
    planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=ms,
                          xy_res=0.25, th_res=math.radians(3.0),
                          clearance=0.15, max_cusps=2, clear_target=0.30)
    res = planner.plan(tuple(st.staging_pose), st.goal_pose)
    print(f'=== max_steer={ms}: success={res.success} len={res.length:.2f} '
          f'nsw={res.n_switch} reason={res.reason} ===')
    if not res.success:
        continue
    path = res.path
    gears = [p[3] for p in path]
    sw = next((i for i in range(1, len(path)) if gears[i] != gears[0]), None)
    seglen = [math.hypot(path[i + 1][0] - path[i][0],
                         path[i + 1][1] - path[i][1])
              for i in range(len(path) - 1)]
    print(f'  D 段 {sum(seglen[:sw]):.2f} m，R 段 {sum(seglen[sw:]):.2f} m，'
          f'{len(path)} 点')
    # 逐点：yaw 字段 vs 折线切线（前后邻点弦向）
    acc = 0.0
    bias = []
    print('   i    s     yaw字段°  弦切°   偏差°   gear  κ字段')
    for i, p in enumerate(path):
        if i:
            acc += seglen[i - 1]
        if i == 0 or i == len(path) - 1:
            continue
        dx = path[i + 1][0] - path[i - 1][0]
        dy = path[i + 1][1] - path[i - 1][1]
        chord = math.degrees(math.atan2(dy, dx))
        d = math.degrees(wrap(p[2] - math.radians(chord)))
        k = p[4] if len(p) >= 5 else 0.0
        bias.append(d)
        if i % 3 == 0:
            print(f'{i:4d} {acc:6.2f} {math.degrees(wrap(p[2])):8.2f} '
                  f'{chord:7.2f} {d:+7.2f} {p[3]:4d} {k:+.3f}')
    # 反转邻点弦（R 段折线方向与 yaw 相反）
    print(f'  yaw-弦偏差：mean {sum(bias)/len(bias):+.2f}°, '
          f'min {min(bias):+.2f}°, max {max(bias):+.2f}°')
    # R 段：折线切线 = 弦向 + 180°（倒车段 yaw 背对行进方向）
    bias_r = []
    for i, p in enumerate(path):
        if i == 0 or i == len(path) - 1 or i < sw:
            continue
        dx = path[i + 1][0] - path[i - 1][0]
        dy = path[i + 1][1] - path[i - 1][1]
        chord = math.degrees(math.atan2(dy, dx))
        d = math.degrees(wrap(p[2] - math.radians(chord) - math.pi))
        bias_r.append(d)
    if bias_r:
        print(f'  R 段 yaw-(弦+180°)：mean {sum(bias_r)/len(bias_r):+.2f}°, '
              f'min {min(bias_r):+.2f}°, max {max(bias_r):+.2f}°')
