#!/usr/bin/env python3
"""读 TRACE_CSV，按固定间隔打印内部量 + 关键事件（换挡/卡死）附近的密集段。"""
import csv
import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/e2_trace.csv'
rows = list(csv.reader(open(path)))[1:]
print(f'rows={len(rows)}  t=[{float(rows[0][0]):.2f} .. {float(rows[-1][0]):.2f}]')
print('   t     x      y      yaw      e      phi     kappa  gear  remaining')

gears = [r[7] for r in rows]
events = [i for i in range(1, len(rows)) if gears[i] != gears[i - 1]]
last = float(rows[-1][0])
sel = set()
for i, r in enumerate(rows):
    t = float(r[0])
    if abs(t * 2 - round(t * 2)) < 1e-9 and round(t * 2) % 2 == 0:  # 每 1 s
        sel.add(i)
for ev in events:
    sel.update(range(max(0, ev - 6), min(len(rows), ev + 10)))
for _ in range(20):
    sel.add(len(rows) - 1 - _)
prev_t = None
for i in sorted(sel):
    r = rows[i]
    t = float(r[0])
    mark = ''
    if i in events:
        mark = '   <-- 换挡'
    if prev_t is not None and t - prev_t > 0.5:
        print('    ......')
    print(f'{t:6.2f} {float(r[1]):7.3f} {float(r[2]):7.3f} {float(r[3]):8.3f} '
          f'{float(r[4]):7.3f} {float(r[5]):+7.3f} {float(r[6]):+7.4f}  '
          f'{r[7]:>3s}  {float(r[8]):8.3f}{mark}')
    prev_t = t
