#!/usr/bin/env python3
"""打印 E2 路径换挡点（cusp）前后各若干点：位置/航向/档位/曲率/累计弧长。
用法：python3 scripts/probe_e2_cusp.py [车位] [前后点数]"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from parking_planner.hybrid_astar import HybridAStar                    # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE                # noqa: E402
from parking_planner.route_planner import RouteGraph, default_graph_path  # noqa: E402

from verify_park_sim import (CLEARANCE, PLAN_MAX_STEER, TH_RES, XY_RES,  # noqa: E402
                             WORLD, SPAWN, handoff_pose, plan_path)

layout = Layout.from_world(WORLD).to_frame(SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
tid = sys.argv[1] if len(sys.argv) > 1 else 'E2'
pad = int(sys.argv[2]) if len(sys.argv) > 2 else 8
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
gears = [p[3] for p in path]
sw = [k for k in range(1, len(gears)) if gears[k] != gears[k - 1]]
print(f'{tid}: len={res.length:.2f} n_sw={res.n_switch} points={len(path)} '
      f'gear-change idx={sw}')
s = 0.0
cum = [0.0]
for a, b in zip(path[:-1], path[1:]):
    s += math.hypot(b[0] - a[0], b[1] - a[1])
    cum.append(s)
for k in sw:
    lo, hi = max(0, k - pad), min(len(path), k + pad + 1)
    print(f'--- cusp @ idx {k} (s={cum[k]:.2f} m) ---')
    print(f'{"i":>4s} {"x":>8s} {"y":>8s} {"yaw°":>8s} {"g":>2s} {"k":>7s} '
          f'{"s":>6s} {"seg_len":>7s} {"seg_dir°":>8s}')
    for i in range(lo, hi):
        seglen = cum[i + 1] - cum[i] if i + 1 < len(path) else float('nan')
        sdir = math.degrees(math.atan2(path[i + 1][1] - path[i][1],
                                       path[i + 1][0] - path[i][0])) \
            if i + 1 < len(path) else float('nan')
        print(f'{i:4d} {path[i][0]:8.3f} {path[i][1]:8.3f} '
              f'{math.degrees(path[i][2]):8.2f} {path[i][3]:+2d} '
              f'{path[i][4]:+7.4f} {cum[i]:6.2f} {seglen:7.3f} {sdir:8.2f}')
