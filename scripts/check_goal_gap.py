#!/usr/bin/env python3
"""逐车位体检：目标位姿处**真实车体矩形**到最近障碍的间隙 + 是否可作目标。

不启 Gazebo、不规划，只做几何判断 —— 用来一眼看出「哪些车位根本不能当目标」。
判据来自 `geometry.body_gap`（真实矩形，不是规划器的近似圆）。

用法：
    python3 scripts/check_goal_gap.py          # 只列垂直车位
    python3 scripts/check_goal_gap.py all      # 连平行车位一起列
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
    show_all = len(sys.argv) > 1 and sys.argv[1] == 'all'
    lay = Layout.from_world(WORLD).to_frame(SPAWN)
    print(f'{"车位":5s} {"类型":13s} {"目标位姿":24s} {"车体最小间隙":12s} 判定')
    bad = []
    for st in lay.stalls:
        gx, gy, gth = st.goal_pose
        gap = body_gap((gx, gy, gth), lay.obstacles)
        usable = gap > 0.02
        if gap <= 0.02:
            bad.append(st.id)
        if show_all or st.kind == 'perpendicular':
            print(f'{st.id:5s} {st.kind:13s} '
                  f'({gx:6.2f},{gy:6.2f},{math.degrees(gth):7.1f}) '
                  f'{gap:+10.3f} m  {"可用" if usable else "**被占/重叠**"}')
    n = len(lay.stalls)
    print(f'\n{n} 个车位中 {len(bad)} 个目标位姿不可用：{bad}')


if __name__ == '__main__':
    main()
