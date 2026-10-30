#!/usr/bin/env python3
"""一次性探针：把 E3 a1 护栏报告的障碍 (lon=+2.28, lat=-0.97) 反算到地图帧。

时间基：位姿 CSV 是仿真时钟（t_stamp），节点日志是墙钟。标定方法：
终点位姿首次到达（停车完成 wall=1793153043.8）→ 用差值把护栏触发墙钟
（1793152982~2989）换到仿真时钟。
"""
import csv
import math

CSV = '/tmp/pose_E3_a1.csv'
WALL_DONE = 1793153043.8
WALL_GUARD = (1793152982.7, 1793152989.1)
LON, LAT = 2.28, -0.97          # 2982 那一拍的护栏最近点（车体系，y 左正）

rows = []
with open(CSV) as f:
    for r in csv.reader(f):
        if r and r[0] == 'est':
            rows.append((float(r[2]), float(r[3]), float(r[4]), float(r[5])))

fx, fy = rows[-1][1], rows[-1][2]
print(f'final est pose: ({fx:.2f},{fy:.2f})  rows={len(rows)}')

arr = None
for t, x, y, yaw in rows:
    if math.hypot(x - fx, y - fy) < 0.15:
        arr = t
        break
print(f'arrival (first within 0.15 m of final): sim_t={arr}')

if arr is None:
    raise SystemExit('no arrival found')

lo = arr + (WALL_GUARD[0] - WALL_DONE)
hi = arr + (WALL_GUARD[1] - WALL_DONE)
print(f'guard window in sim time: [{lo:.1f}, {hi:.1f}]')

for t, x, y, yaw in rows:
    if lo - 6 <= t <= hi + 6:
        c, s = math.cos(yaw), math.sin(yaw)
        ox = x + c * LON - s * LAT
        oy = y + s * LON + c * LAT
        if abs(t - (lo + hi) / 2) < 1.1 or (lo - 6 <= t <= lo - 4):
            print(f'sim={t:7.2f} ego=({x:6.2f},{y:6.2f},{math.degrees(yaw):7.2f}deg)'
                  f'  obstacle_map=({ox:6.2f},{oy:6.2f})')
