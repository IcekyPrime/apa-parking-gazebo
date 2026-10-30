#!/usr/bin/env python3
import csv
import math
import os
import sys

WS = os.path.expanduser('~/apa_ws')
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
from parking_planner.lot_layout import Layout

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds', 'city_parking_lot.sdf')
lay = Layout.from_world(WORLD).to_frame((-30.0, 15.65, 0.0))
st = lay.find_stall('E2')
print('staging(map)=', tuple(round(v, 2) for v in st.staging_pose))
print('goal(map)=', tuple(round(v, 2) for v in st.goal_pose))
rows = list(csv.DictReader(open('/tmp/pose_E2_a1.csv')))
gt = [(float(r['x']), float(r['y'])) for r in rows if r['src'] == 'gt']
xs = [p[0] for p in gt]
ys = [p[1] for p in gt]
print('gt x range', round(min(xs), 1), round(max(xs), 1),
      ' y range', round(min(ys), 1), round(max(ys), 1))
dmin, at = min(((math.hypot(x - st.staging_pose[0], y - st.staging_pose[1]), i)
                for i, (x, y) in enumerate(gt)), key=lambda t: t[0])
print('min dist gt->staging =', round(dmin, 2), 'at frame', at)
# 距离 < 3 m 的帧数
near = [i for i, (x, y) in enumerate(gt)
        if math.hypot(x - st.staging_pose[0], y - st.staging_pose[1]) < 3.0]
print('frames within 3 m of staging:', len(near))
if near:
    print('first near frame idx', near[0], 't=',
          [r['t_stamp'] for r in rows if r['src'] == 'gt'][near[0]])
