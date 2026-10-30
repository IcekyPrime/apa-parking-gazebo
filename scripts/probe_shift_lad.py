#!/usr/bin/env python3
"""分析 TRACE_CSV：找预瞄距离塌缩（换挡点锚定）与 κ 顶满的时间关系。"""
import sys

path = sys.argv[1] if len(sys.argv) > 1 else '/tmp/e2_trace.csv'
tail = float(sys.argv[2]) if len(sys.argv) > 2 else 0.0
print(f'{"t":>6s} {"e":>7s} {"phi":>7s} {"kappa":>8s} {"gear":>4s} {"rem":>7s} '
      f'{"la_d":>6s} {"k_pp":>8s}')
with open(path) as fp:
    next(fp)
    for line in fp:
        p = line.strip().split(',')
        if len(p) < 11:
            continue
        t = float(p[0])
        if t < tail:
            continue
        print(f'{t:6.2f} {float(p[4]):+7.3f} {float(p[5]):+7.2f} {float(p[6]):+8.4f} '
              f'{p[7]:>4s} {float(p[8]):7.3f} {float(p[9]):6.3f} {float(p[10]):+8.4f}')
