#!/usr/bin/env python3
"""打印 e2_pose_analysis.csv 的逐秒摘要：s / e_true / w_true / yaw。"""
import csv
import math
import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/e2_pose_analysis.csv'
rows = list(csv.DictReader(open(path)))
print(f'{len(rows)} frames')
print('t_rel   s      e_true   yaw°     w_true    thr°     φ°')
last_t = None
for r in rows:
    t = float(r['t_rel'])
    if last_t is not None and t - last_t < 0.98:
        continue
    last_t = t
    print(f"{t:6.2f} {float(r['s_gt']):6.2f} {float(r['e_true']):+7.3f} "
          f"{math.degrees(float(r['yaw_gt'])):+8.2f} {float(r['w_true']):+.4f} "
          f"{float(r['thr_deg']):+8.2f} {float(r['phi_deg']):+6.2f}")
