#!/usr/bin/env python3
"""从 Gazebo world 生成先验车位布局 `lot_layout.yaml`。

用法：
    ros2 run parking_planner make_layout \
        --world ~/apa_ws/src/parking_lot_city/worlds/city_parking_lot.sdf \
        --out   ~/apa_ws/src/parking_planner/config/lot_layout.yaml
"""

import argparse
import os
import sys

from .lot_layout import Layout


def main(args=None):
    default_world = os.path.expanduser(
        '~/apa_ws/src/parking_lot_city/worlds/city_parking_lot.sdf')
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_out = os.path.join(here, 'config', 'lot_layout.yaml')

    ap = argparse.ArgumentParser(description='从 world 生成车位布局')
    ap.add_argument('--world', default=default_world)
    ap.add_argument('--out', default=default_out)
    ap.add_argument('--resolution', type=float, default=0.10)
    ns = ap.parse_args(args if args is not None else sys.argv[1:])

    if not os.path.exists(ns.world):
        print(f'[make_layout] world 不存在：{ns.world}')
        return 1

    layout = Layout.from_world(ns.world, ns.resolution)
    layout.save(ns.out)

    print(f'[make_layout] 已写出 {ns.out}')
    print(f'  坐标系: {layout.meta.get("frame")}  '
          f'出生位姿: {layout.meta.get("spawn")}')
    print(f'  ROI: {[round(v, 2) for v in layout.roi or []]}')
    print(f'  障碍物 {len(layout.obstacles)} 个，车位 {len(layout.stalls)} 个')
    per = [s for s in layout.stalls if s.kind == 'perpendicular']
    par = [s for s in layout.stalls if s.kind == 'parallel']
    print(f'  垂直车位 {len(per)} 个，平行车位 {len(par)} 个')
    for s in layout.stalls[:3]:
        print(f'    例：{s}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
