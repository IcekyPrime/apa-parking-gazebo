#!/usr/bin/env python3
"""读 TRACE_CSV，按档位分段打印关键帧（含带符号 e / thr / kref / 真实位姿误差）。

用法：python3 scripts/read_trace.py /tmp/E2_seed5.csv [gear过滤(如-1)] [每n帧取1] [起始t]
"""
import csv
import math
import sys

fp = sys.argv[1]
gear_f = int(sys.argv[2]) if len(sys.argv) > 2 else None
step = int(sys.argv[3]) if len(sys.argv) > 3 else 4
t0 = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0

rows = list(csv.DictReader(open(fp)))
print(f'{"t":>6s} {"x":>7s} {"y":>7s} {"yaw":>7s} {"e":>6s} {"phi":>7s} '
      f'{"kappa":>7s} {"g":>2s} {"rem":>6s} {"la_d":>5s} {"k_pp":>7s} '
      f'{"e_sig":>6s} {"thr":>7s} {"kref":>7s} {"e_t":>6s} {"phi_t":>7s}')
k = 0
for r in rows:
    g = int(r['gear'])
    t = float(r['t'])
    if t < t0:
        continue
    if gear_f is not None and g != gear_f:
        k += 1
        continue
    if k % step:
        k += 1
        continue
    k += 1
    print(f"{t:6.2f} {float(r['x']):7.2f} {float(r['y']):7.2f} "
          f"{math.degrees(float(r['yaw'])):7.2f} {float(r['e_meas']):6.3f} "
          f"{math.degrees(float(r['phi'])):7.2f} {float(r['kappa']):7.4f} "
          f"{g:2d} {float(r['remaining']):6.2f} {float(r['la_d']):5.2f} "
          f"{float(r['k_pp']):7.4f} "
          f"{float(r.get('e_sig', 0)):6.3f} {math.degrees(float(r.get('thr', 0))):7.2f} "
          f"{float(r.get('kref', 0)):7.4f} "
          f"{float(r.get('e_true', 0)):6.3f} "
          f"{math.degrees(float(r.get('phi_true', 0))):7.2f}")
