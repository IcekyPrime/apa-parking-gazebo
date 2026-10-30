#!/usr/bin/env python3
"""解剖 verify_park_sim 给 E2 生成的路径：末尾点的档位/曲率字段。"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.hybrid_astar import HybridAStar                    # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE                # noqa: E402
from parking_planner.route_planner import RouteGraph, default_graph_path  # noqa: E402

# 复用 verify_park_sim 里的构造（同一个目录 import）
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_park_sim import (CLEARANCE, PLAN_MAX_STEER, TH_RES, XY_RES,  # noqa: E402
                             WORLD, SPAWN, handoff_pose, plan_path)

layout = Layout.from_world(WORLD).to_frame(SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
tid = sys.argv[1] if len(sys.argv) > 1 else 'E2'
st = layout.find_stall(tid)
route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
hand = handoff_pose(route, 0.0, 0.0, 0.0, target=st.staging_pose)
pts = [hand, st.goal_pose, st.staging_pose]
cx = sum(p[0] for p in pts) / 3.0
cy = sum(p[1] for p in pts) / 3.0
half = 25.0
grid = layout.occupancy_grid(XY_RES / 2.0,
                             roi=(cx - half, cx + half, cy - half, cy + half))
planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=PLAN_MAX_STEER,
                      xy_res=XY_RES, th_res=TH_RES,
                      clearance=CLEARANCE if st.kind == 'perpendicular' else 0.0,
                      max_cusps=2 if st.kind == 'perpendicular' else 3)
res, path, used = plan_path(planner, hand, st)
print(f'{tid}: reason={res.reason} len={res.length:.2f} n_sw={res.n_switch} '
      f'used_stage={used} points={len(path)}')
gears = [p[3] for p in path]
changes = [(k, gears[k - 1], gears[k]) for k in range(1, len(gears))
           if gears[k] != gears[k - 1]]
print(f'档位变化点: {changes}')
print(f'末 12 点 (x, y, yaw°, gear, κ):')
for p in path[-12:]:
    print(f'  ({p[0]:8.3f}, {p[1]:8.3f}, {math.degrees(p[2]):7.2f}°, '
          f'{p[3]:+d}, {p[4]:+.4f})')
seg_sum = sum(math.hypot(b[0] - a[0], b[1] - a[1])
              for a, b in zip(path[:-1], path[1:]))
print(f'折线总长 = {seg_sum:.2f}')
