#!/usr/bin/env python3
"""一次性探针：地图 (32.1,-37.9) 附近 3 m 内的静态障碍（E3 护栏最近点反算）。"""
import math
import os
import sys

WS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import Rect  # noqa: E402
from parking_planner.lot_layout import Layout  # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
SPAWN = (-30.0, 15.65, 0.0)
PX, PY = 32.1, -37.9


def rect_gap(r, px, py):
    c, s = math.cos(r.yaw), math.sin(r.yaw)
    dx, dy = px - r.cx, py - r.cy
    lx, ly = c * dx + s * dy, -s * dx + c * dy
    qx = max(abs(lx) - r.hl, 0.0)
    qy = max(abs(ly) - r.hw, 0.0)
    return math.hypot(qx, qy)


layout = Layout.from_world(WORLD).to_frame(SPAWN)
print(f'静态障碍（含车位车）到 ({PX},{PY}) 的距离 < 3.0 m：')
found = False
for ob in layout.obstacles:
    if isinstance(ob, Rect):
        d = rect_gap(ob, PX, PY)
        if d < 3.0:
            found = True
            print(f'  Rect c=({ob.cx:6.2f},{ob.cy:6.2f}) yaw={ob.yaw:5.2f} '
                  f'hl={ob.hl:4.2f} hw={ob.hw:4.2f} d={d:.2f} '
                  f'name={getattr(ob, "name", "?")}')
if not found:
    print('  （无 —— 3 m 内没有静态障碍）')

print('\n各排车位目标/预备位姿（map 帧）：')
for tid in ('W1', 'E0', 'E1', 'E2', 'E3', 'E4'):
    try:
        st = layout.find_stall(tid)
        gx, gy, gyaw = st.goal_pose
        sp = st.staging_pose
        print(f'{tid}: goal=({gx:6.2f},{gy:6.2f},{math.degrees(gyaw):6.1f}deg)'
              f'  staging=({sp[0]:6.2f},{sp[1]:6.2f},{math.degrees(sp[2]):6.1f}deg)')
    except Exception as e:
        print(f'{tid}: {e}')
