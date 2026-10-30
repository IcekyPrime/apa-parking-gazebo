#!/usr/bin/env python3
"""打印离线重建 E2 锚定解的逐点几何（s / yaw / κ / gear），对照真机日志参考yaw。"""
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
grid = layout.occupancy_grid(0.125, roi=(cx - half, cx + half, cy - half, cy + half))
planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=0.42,
                      xy_res=0.25, th_res=math.radians(3.0),
                      clearance=0.15, max_cusps=2, clear_target=0.30)
res = planner.plan(tuple(st.staging_pose), st.goal_pose)
print('success', res.success, 'len', round(res.length, 2),
      'nsw', res.n_switch, 'reason', res.reason)
path = res.path
# 逐点：s、世界坐标、yaw、gear、kappa
acc = 0.0
print('  i    s     x(map)  y(map)   yaw°   gear  kappa')
for i, p in enumerate(path):
    if i:
        acc += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
    k = p[4] if len(p) >= 5 else 0.0
    if i % 3 == 0 or i == len(path) - 1 or path[i][3] != path[i - 1][3]:
        print(f'{i:4d} {acc:6.2f} {p[0] + SPAWN[0]:7.2f} {p[1] + SPAWN[1]:7.2f} '
              f'{math.degrees(wrap(p[2])):8.2f} {p[3]:4d} {k:+.3f}')
# 段级曲率统计
gears = [p[3] for p in path]
sw = next((i for i in range(1, len(path)) if gears[i] != gears[0]), None)
seglen = [math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
          for i in range(len(path) - 1)]
d_leg = sum(seglen[:sw])
r_leg = sum(seglen[sw:])
d_yaw = wrap(path[sw][2] - path[0][2])
r_yaw = wrap(path[-1][2] - path[sw][2])
print(f'D 段 {d_leg:.2f} m 转 {math.degrees(d_yaw):+.1f}° '
      f'→ κ={d_yaw / max(d_leg, 1e-6):+.4f}')
print(f'R 段 {r_leg:.2f} m 转 {math.degrees(r_yaw):+.1f}° '
      f'→ κ={r_yaw / max(r_leg, 1e-6):+.4f}')
print(f'终点 yaw {math.degrees(wrap(path[-1][2])):.2f}°，'
      f'goal yaw {math.degrees(wrap(st.goal_pose[2])):.2f}°')
