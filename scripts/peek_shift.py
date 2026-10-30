#!/usr/bin/env python3
"""找到 TRACE_CSV 里第一次换挡（gear 1→-1）的帧，打印前后窗口。
用法：python3 scripts/peek_shift.py <csv> [前帧数] [后帧数]"""
import csv
import sys

fp = sys.argv[1]
pre = int(sys.argv[2]) if len(sys.argv) > 2 else 14
post = int(sys.argv[3]) if len(sys.argv) > 3 else 4

rows = list(csv.DictReader(open(fp)))
hit = None
for i, r in enumerate(rows):
    if int(r['gear']) == -1 and i > 0 and int(rows[i - 1]['gear']) == 1:
        hit = i
        break
if hit is None:
    print('没有找到 1→-1 换挡帧（gear 序列：'
          f'{sorted(set(int(r["gear"]) for r in rows))}）')
    sys.exit(0)
print(f'shift @ frame {hit}, t={rows[hit]["t"]}')
cols = ['t', 'seg', 'remaining', 'kref', 'kappa', 'la_d', 'k_pp',
        'e_meas', 'phi', 'x', 'y', 'thr']
print(' '.join(f'{c:>8s}' for c in cols))
for j in range(max(0, hit - pre), min(len(rows), hit + post)):
    r = rows[j]
    print(' '.join(f'{r.get(c, "")[:8]:>8s}' for c in cols))
