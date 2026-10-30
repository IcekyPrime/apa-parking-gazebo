#!/usr/bin/env python3
"""看 pose_E2_a1.csv 的 gt 流在入位起步前后的真实运动（位置/速度/航向）。"""
import csv
import math
import sys

csv_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/pose_E2_a1.csv'
t0 = float(sys.argv[2]) if len(sys.argv) > 2 else 107.35
t_lo = float(sys.argv[3]) if len(sys.argv) > 3 else 105.5
t_hi = float(sys.argv[4]) if len(sys.argv) > 4 else 122.0
step = float(sys.argv[5]) if len(sys.argv) > 5 else 0.4

rows = [r for r in csv.DictReader(open(csv_path)) if r['src'] == 'gt']
pts = [(float(r['t_stamp']), float(r['x']), float(r['y']), float(r['yaw']))
       for r in rows]
pts.sort()
prev = None
nxt = 0.0
print('t_abs   t_rel    x       y       yaw°    v(m/s)')
for t, x, y, yaw in pts:
    if not (t_lo <= t <= t_hi):
        prev = (t, x, y, yaw)
        continue
    if prev is not None:
        dt = t - prev[0]
        v = math.hypot(x - prev[1], y - prev[2]) / dt if dt > 1e-3 else 0.0
    else:
        v = 0.0
    tr = t - t0
    if tr >= nxt - 1e-9:
        print(f'{t:7.2f} {tr:+6.2f} {x:7.2f} {y:7.2f} '
              f'{math.degrees(yaw):+8.2f} {v:6.3f}')
        nxt += step
    prev = (t, x, y, yaw)
