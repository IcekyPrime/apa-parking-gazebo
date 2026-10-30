#!/usr/bin/env python3
"""输出 verify_park_sim 给 E2 生成的整条路径剖面：沿弧长每 0.5 m 一行
（档位 / 路径 yaw / RS 曲率字段 c / 跟踪器口径 κ_ref=-c·gear 反号后的值），
并打印换挡点位姿与终点。"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from parking_planner.lot_layout import Layout, WHEELBASE          # noqa: E402
from parking_planner.route_planner import (RouteGraph,            # noqa: E402
                                           default_graph_path)
from verify_park_sim import (CLEARANCE, PLAN_MAX_STEER, TH_RES,   # noqa: E402
                             XY_RES, WORLD, SPAWN, handoff_pose, plan_path)

layout = Layout.from_world(WORLD).to_frame(SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
tid = sys.argv[1] if len(sys.argv) > 1 else 'E2'
st = layout.find_stall(tid)
route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
hand = handoff_pose(route, 0.0, 0.0, 0.0, target=st.staging_pose)
pts = [hand, st.goal_pose, st.staging_pose]
cx = sum(p[0] for p in pts) / 3.0
cy = sum(p[1] for p in pts) / 3.0
grid = layout.occupancy_grid(XY_RES / 2.0,
                             roi=(cx - 25.0, cx + 25.0, cy - 25.0, cy + 25.0))
from parking_planner.hybrid_astar import HybridAStar              # noqa: E402
planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=PLAN_MAX_STEER,
                      xy_res=XY_RES, th_res=TH_RES,
                      clearance=CLEARANCE if st.kind == 'perpendicular' else 0.0,
                      max_cusps=2 if st.kind == 'perpendicular' else 3)
res, path, used = plan_path(planner, hand, st)
print(f'{tid}: reason={res.reason} len={res.length:.2f} n_sw={res.n_switch} '
      f'pts={len(path)}')
print(f'hand=({hand[0]:.2f},{hand[1]:.2f},{math.degrees(hand[2]):.1f}°) '
      f'stage=({st.staging_pose[0]:.2f},{st.staging_pose[1]:.2f},'
      f'{math.degrees(st.staging_pose[2]):.1f}°) '
      f'goal=({st.goal_pose[0]:.2f},{st.goal_pose[1]:.2f},'
      f'{math.degrees(st.goal_pose[2]):.1f}°)')
print(f'{"s":>5s} {"x":>7s} {"y":>7s} {"yaw":>7s} {"g":>2s} {"c":>7s} '
      f'{"k_ref":>7s}')
s = 0.0
marks = {0.0: 'start'}
for k in range(1, len(path)):
    if path[k][3] != path[k - 1][3]:
        marks[s] = f'cusp→{path[k][3]:+d}'
nxt = 0.5
for k, p in enumerate(path):
    if k:
        s += math.hypot(p[0] - path[k - 1][0], p[1] - path[k - 1][1])
    while s >= nxt:
        c = p[4]
        kref = -c if p[3] < 0 else c
        mark = marks.get(round(s, 6), '')
        print(f'{s:5.2f} {p[0]:7.2f} {p[1]:7.2f} {math.degrees(p[2]):7.2f} '
              f'{p[3]:+2d} {c:+7.4f} {kref:+7.4f} {mark}')
        nxt += 0.5
p = path[-1]
print(f'{s:5.2f} {p[0]:7.2f} {p[1]:7.2f} {math.degrees(p[2]):7.2f} '
      f'{p[3]:+2d} {p[4]:+7.4f} {-p[4] if p[3] < 0 else p[4]:+7.4f} end')
