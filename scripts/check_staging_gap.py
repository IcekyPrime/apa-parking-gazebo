#!/usr/bin/env python3
"""检查每个车位的**预备位姿**处车体到最近障碍的间隙。

真机 W4 在预备位姿附近被「入位贴合护栏」拦死（间隙 0.02 m）。如果预备位姿本身
就贴着邻车，那么任何从它出发的入位路径都会被自己的护栏挡住 —— 那是布局/几何
问题，不是控制问题。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import body_gap      # noqa: E402
from parking_planner.lot_layout import Layout      # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds', 'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)


def main():
    lay = Layout.from_world(WORLD).to_frame(SPAWN)
    print(f'{"车位":5s} {"预备位姿":26s} {"预备处车体间隙":14s} {"目标处间隙":10s}')
    bad = []
    for st in lay.stalls:
        sx, sy, sth = st.staging_pose
        g = body_gap(st.staging_pose, lay.obstacles)
        gg = body_gap(st.goal_pose, lay.obstacles)
        flag = ''
        if g < 0.10:
            flag = '  <== 预备位姿贴着障碍'
            bad.append(st.id)
        print(f'{st.id:5s} ({sx:6.2f},{sy:6.2f},{math.degrees(sth):7.1f}) '
              f'{g:+12.3f} m {gg:+9.3f} m{flag}')
    print(f'\n预备位姿间隙 <0.10 m 的车位：{bad if bad else "无"}')


if __name__ == '__main__':
    main()
