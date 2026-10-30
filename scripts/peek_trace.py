#!/usr/bin/env python3
"""打印 verify_park_sim TRACE_CSV 的逐米摘要，对照真机 e 漂移率。"""
import csv
import math
import sys

rows = list(csv.DictReader(open(sys.argv[1] if len(sys.argv) > 1 else '/tmp/e2_fixedsim_oldctl.csv')))
print(f'{len(rows)} frames')
print('  s      e_true  phi_true  kappa   k_pp   gear  la_d  rem')
last_s = None
for r in rows:
    s = float(r['remaining'])
    if last_s is not None and last_s - s < 0.98:
        continue
    last_s = s
    print(f"{s:6.2f} {float(r['e_true']):+7.3f} {math.degrees(float(r['phi_true'])):+8.2f} "
          f"{float(r['kappa']):+.3f} {float(r['k_pp']):+.3f} {r['gear']:>3s} "
          f"{float(r['la_d']):5.2f} {float(r['remaining']):6.2f}")
