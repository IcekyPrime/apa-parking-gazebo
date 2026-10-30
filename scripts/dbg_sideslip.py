#!/usr/bin/env python3
"""测 /odom_gt 流的车体侧偏角 β = 速度方向 - 航向。

若 β 在弧线段稳定为 -10° 量级（而直线段 ≈0），则「e 单调发散但 φ 正常」
的真因是车辆模型侧滑，不是跟踪律。
"""
import csv
import math
import sys

csv_path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/pose_E2_a1.csv'
win = float(sys.argv[2]) if len(sys.argv) > 2 else 0.5   # 差分基线 (s)

rows = [r for r in csv.DictReader(open(csv_path)) if r['src'] == 'gt']
pts = [(float(r['t_stamp']), float(r['x']), float(r['y']), float(r['yaw']))
       for r in rows]
pts.sort()

print(f'  t      x      y      yaw°    v     β(速度方向-航向)°')
i = 0
nxt_t = 100.0
while i < len(pts):
    t0 = pts[i][0]
    if t0 < nxt_t:
        i += 1
        continue
    # 找 t0+win 处的帧
    j = i
    while j + 1 < len(pts) and pts[j + 1][0] < t0 + win:
        j += 1
    if j > i:
        dt = pts[j][0] - pts[i][0]
        dx = pts[j][1] - pts[i][1]
        dy = pts[j][2] - pts[i][2]
        d = math.hypot(dx, dy)
        if dt > 1e-3 and d > 0.05:
            v = d / dt
            vdir = math.atan2(dy, dx)
            # 航向取窗口中点（线性插值）
            tm = (pts[i][0] + pts[j][0]) / 2.0
            yaw_m = None
            for k in range(i, j):
                if pts[k][0] <= tm <= pts[k + 1][0]:
                    f = (tm - pts[k][0]) / (pts[k + 1][0] - pts[k][0])
                    yaw_m = pts[k][3] + f * (pts[k + 1][3] - pts[k][3])
                    break
            if yaw_m is None:
                yaw_m = pts[i][3]
            beta = math.degrees(math.atan2(
                math.sin(vdir - yaw_m), math.cos(vdir - yaw_m)))
            print(f'{t0:7.2f} {pts[i][1]:6.2f} {pts[i][2]:6.2f} '
                  f'{math.degrees(yaw_m):+7.2f} {v:5.2f}  {beta:+7.2f}')
    nxt_t = t0 + 1.0
    i += 1
