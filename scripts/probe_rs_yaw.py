#!/usr/bin/env python3
"""验证 RS 采样里倒车段的 yaw 存储约定：yaw 是车头朝向还是行进方向。"""
import math
import os
import sys

WS = os.path.expanduser('~/apa_ws')
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
from parking_planner.reeds_shepp import reeds_shepp          # noqa: E402
from parking_planner.geometry import wrap                     # noqa: E402

sol = reeds_shepp((0.0, 0.0, 0.0), (-4.0, -3.0, math.pi), radius=6.0)
print('path:', sol)
pts = sol.sample(0.15)
prev = None
for i, q in enumerate(pts):
    if i == 0:
        prev = q
        continue
    tan = math.atan2(q[1] - prev[1], q[0] - prev[0])
    d = wrap(q[2] - tan)
    if i % 8 == 0 or q[3] != prev[3]:
        print(f'i={i:3d} dir={q[3]:+d} yaw={math.degrees(q[2]):+7.1f} '
              f'tan={math.degrees(tan):+7.1f} yaw-tan={math.degrees(d):+7.1f}')
    prev = q
