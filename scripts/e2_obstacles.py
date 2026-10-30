#!/usr/bin/env python3
"""列出 E2 目标/预备位姿附近的静态障碍与距离，弄清执行间隙瓶颈在哪一侧。"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
from parking_planner.geometry import body_gap       # noqa: E402
from parking_planner.lot_layout import Layout       # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds', 'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)

layout = Layout.from_world(WORLD).to_frame(SPAWN)
st = layout.find_stall('E2')
gx, gy, gyaw = st.goal_pose
sx, sy, syaw = st.staging_pose
print(f'E2 goal   = ({gx:.2f}, {gy:.2f}, {math.degrees(gyaw):.0f}°)')
print(f'E2 stage  = ({sx:.2f}, {sy:.2f}, {math.degrees(syaw):.0f}°)')
print(f'障碍总数  = {len(layout.obstacles)}')

def near(p, r=12.0):
    out = []
    for ob in layout.obstacles:
        (ox, oy), (hw, hh), oyaw = ob
        d = math.hypot(ox - p[0], oy - p[1])
        if d < r:
            g = body_gap(p, [ob])
            out.append((d, g, ob))
    out.sort()
    return out

for name, p in (('goal', st.goal_pose), ('stage', st.staging_pose)):
    print(f'\n--- 距 {name} 12 m 内的障碍（按距离） ---')
    for d, g, ob in near(p)[:12]:
        (ox, oy), (hw, hh), oyaw = ob
        print(f'  d={d:5.2f} m  车体间隙={g:+6.2f} m  '
              f'中心=({ox:7.2f},{oy:7.2f}) 半宽={2*hw:.2f}x{2*hh:.2f} '
              f'yaw={math.degrees(oyaw):.0f}°')

# 过道边界：E2 排的过道中心线 x（stage 的 x）与两侧
print(f'\nstage x={sx:.2f}（过道中心线附近）')
