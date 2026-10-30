#!/usr/bin/env python3
"""打印 E2 路径**曲率变号区**（非换挡、曲率符号翻转）前后的点列。
用途：确认变号点两侧是否存在 kappa=0 的采样点（它会让「相邻曲率乘积<0」的
flip 检测失效、并把 k_ref 均值稀释成 0）。
用法：python3 scripts/probe_e2_flip.py [车位] [前后点数]"""
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
pad = int(sys.argv[2]) if len(sys.argv) > 2 else 10
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
print(f'{tid}: len={res.length:.2f} n_sw={res.n_switch} points={len(path)}')
s = 0.0
cum = [0.0]
for a, b in zip(path[:-1], path[1:]):
    s += math.hypot(b[0] - a[0], b[1] - a[1])
    cum.append(s)
# 找「同档位内、曲率符号翻转」的区段：前后两个非零曲率异号
ks = [p[4] for p in path]
nz = [i for i, k in enumerate(ks) if abs(k) > 1e-9]
flips = []
for a, b in zip(nz[:-1], nz[1:]):
    if ks[a] * ks[b] < -1e-12:
        flips.append((a, b))
print(f'curvature flips (nonzero sign change): {flips}')
for a, b in flips:
    lo, hi = max(0, a - pad), min(len(path), b + pad + 1)
    nzero = sum(1 for i in range(a, b + 1) if abs(ks[i]) <= 1e-9)
    print(f'--- flip idx {a}->{b} (s={cum[a]:.2f}->{cum[b]:.2f}, '
          f'zero-pts in between={nzero}) ---')
    print(f'{"i":>4s} {"x":>8s} {"y":>8s} {"yaw°":>8s} {"g":>2s} {"k":>7s} '
          f'{"s":>6s} {"seg_len":>7s}')
    for i in range(lo, hi):
        seglen = cum[i + 1] - cum[i] if i + 1 < len(path) else float('nan')
        print(f'{i:4d} {path[i][0]:8.3f} {path[i][1]:8.3f} '
              f'{math.degrees(path[i][2]):8.2f} {path[i][3]:+2d} '
              f'{path[i][4]:+7.4f} {cum[i]:6.2f} {seglen:7.3f}')
