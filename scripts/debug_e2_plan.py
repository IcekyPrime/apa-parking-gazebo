#!/usr/bin/env python3
"""复刻 verify_park_sim 的构造，解剖 E2 @0.40 的直连解与拼接过程。"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.hybrid_astar import HybridAStar        # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE     # noqa: E402
from parking_planner.planner_node import MIN_SEG_LEN         # noqa: E402
from parking_planner.geometry import wrap                    # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)
XY_RES, TH_RES = 0.25, math.radians(3.0)
CLEARANCE = 0.15
STEER = float(sys.argv[1]) if len(sys.argv) > 1 else 0.40

layout = Layout.from_world(WORLD).to_frame(SPAWN)
st = layout.find_stall('E2')
gx, gy, gyaw = st.goal_pose
sx, sy, syaw = st.staging_pose
pts = [(sx, sy, syaw), (gx, gy, gyaw)]
cx = sum(p[0] for p in pts) / 2.0
cy = sum(p[1] for p in pts) / 2.0
half = max(15.0, max(math.hypot(p[0] - cx, p[1] - cy) for p in pts) + 6.0)
grid = layout.occupancy_grid(XY_RES / 2.0,
                             roi=(cx - half, cx + half, cy - half, cy + half))
print(f'grid roi half={half:.1f}（南边界 y={cy - half:.1f}）')
planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=STEER,
                      xy_res=XY_RES, th_res=TH_RES,
                      clearance=CLEARANCE, max_cusps=2)
print(f'max_iter 默认 = {planner.max_iter}')

res = planner.plan((sx, sy, syaw), (gx, gy, gyaw))
print(f'直连解: success={res.success} reason={res.reason} '
      f'n_switch={res.n_switch} length={res.length:.2f} used_rs={res.used_rs}')
path = list(res.path)
i = next((k for k in range(1, len(path)) if path[k][3] != path[0][3]), None)
if i is not None:
    leg = sum(math.hypot(b[0] - a[0], b[1] - a[1])
              for a, b in zip(path[:i], path[1:i + 1]))
    print(f'首段: 索引 {i}，长度 {leg:.2f} m（MIN_SEG_LEN={MIN_SEG_LEN}），'
          f'起点档位 {path[0][3]}，换挡点档位 {path[i][3]}')
    if leg < MIN_SEG_LEN:
        r = planner.plan(path[i][:3], (gx, gy, gyaw))
        print(f'重规划解: success={r.success} n_switch={r.n_switch} '
              f'length={r.length:.2f}')
        if r.success:
            merged = path[:i + 1] + list(r.path)
            n_sw = sum(1 for a, b in zip(merged, merged[1:])
                       if a[3] != b[3])
            print(f'拼接后实际换挡 = {n_sw}（原解 {res.n_switch}）')
            gears = [p[3] for p in merged[:8]]
            print(f'拼接头部档位序列: {gears}')
else:
    print('无换挡点（0 换挡解）')

# 对照：只许解析解
saved = planner.max_iter
planner.max_iter = 1
r1 = planner.plan((sx, sy, syaw), (gx, gy, gyaw))
planner.max_iter = saved
print(f'\n[对照] max_iter=1 纯解析解: success={r1.success} '
      f'n_switch={r1.n_switch} length={r1.length:.2f}')
p1 = list(r1.path)
j = next((k for k in range(1, len(p1)) if p1[k][3] != p1[0][3]), None)
if j is not None:
    leg1 = sum(math.hypot(b[0] - a[0], b[1] - a[1])
               for a, b in zip(p1[:j], p1[1:j + 1]))
    print(f'解析解首段长 = {leg1:.2f} m，'
          f'换挡点位姿 = ({p1[j][0]:.2f}, {p1[j][1]:.2f}, '
          f'{math.degrees(p1[j][2]):.0f}°, gear={p1[j][3]})')
