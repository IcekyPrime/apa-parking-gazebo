#!/usr/bin/env python3
"""激光车体护栏的纯几何单测（不启 Gazebo）。

`planner_node.corridor_hit` 只做一件事：判断「激光点是否落在车体（外扩余量）
的行驶方向那一半」。它直接决定自车会不会撞上行人或邻车，所以单独测一遍。

点一律用**车体坐标系**给出（x 向前、y 向左，原点在车体中心），测试里再换算到
激光系（激光装在 x=-0.15，见 ego_car/model.sdf）。

用法：`python3 scripts/selftest_guard.py`
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.planner_node import (GUARD_M, HALF_L, HALF_W,   # noqa: E402
                                          LIDAR_DX, body_clear, corridor_hit)

N = 720
AMIN, AMAX = -math.pi, math.pi
INC = (AMAX - AMIN) / N


def scan_with(points):
    """构造一帧扫描。points 用车体系 (x 向前, y 向左)。"""
    ranges = [float('inf')] * N
    for xv, yv in points:
        xs, ys = xv - LIDAR_DX, yv          # 车体系 -> 激光系
        r = math.hypot(xs, ys)
        a = math.atan2(ys, xs)
        idx = int(round((a - AMIN) / INC)) % N
        ranges[idx] = r
    return ranges


def main():
    hl, hw = HALF_L + GUARD_M, HALF_W + GUARD_M
    cases = [
        # 名称, 车体系点, 档位, 期望是否拦截[, ahead]
        ('正前方 2.0 m', [(2.0, 0.0)], +1, True),
        ('正前方 2.8 m', [(2.8, 0.0)], +1, False),
        ('正前方 2.0 m、横偏 2.0 m', [(2.0, 2.0)], +1, False),
        ('正前方 2.0 m、横偏 1.0 m', [(2.0, 1.0)], +1, True),
        ('正前方 1.0 m、横偏 1.2 m', [(1.0, 1.2)], +1, False),
        ('正后方 1.0 m（前进）', [(-1.0, 0.0)], +1, False),
        ('正后方 1.0 m（倒车）', [(-1.0, 0.0)], -1, True),
        ('正后方 1.0 m、横偏 1.5 m（倒车）', [(-1.0, 1.5)], -1, False),
        ('全 inf', [], +1, False),
        # 巡航段（ahead=4.5）的关键回归：平行的马路牙子不能把车冻住
        ('巡航：前方 3.7 m、横偏 1.08 m（平行路缘）', [(3.7, 1.08)], +1, False, 4.5),
        ('巡航：前方 3.7 m、横偏 0.50 m（车道内）', [(3.7, 0.50)], +1, True, 4.5),
        ('巡航：前方 2.0 m、横偏 1.05 m（近处余量）', [(2.0, 1.05)], +1, True, 4.5),
    ]
    print(f'车体模型 {2 * HALF_L:.2f} x {2 * HALF_W:.2f} m，'
          f'护栏判定范围 前/后 {hl:.3f} m、横向 ±{hw:.3f} m，'
          f'激光纵向偏移 {LIDAR_DX:+.2f} m')
    bad = 0
    for c in cases:
        name, pts, gear, want = c[:4]
        ahead = c[4] if len(c) > 4 else None
        got = corridor_hit(scan_with(pts), AMIN, INC, gear, ahead=ahead)
        flag = 'OK ' if got == want else 'FAIL'
        if got != want:
            bad += 1
        print(f'{flag} {name:40s} 期望 {"拦" if want else "放行"} '
              f'实际 {"拦" if got else "放行"}')
    print(f'通过 {len(cases) - bad}/{len(cases)}')

    bad2 = lateral_cases()
    return 1 if (bad or bad2) else 0


def lateral_cases():
    """泊车段的**车体距离**判据（body_clear）单测。"""
    rows = [
        # 名称, 车体系点集, 期望距离(m)（None = 无穷大）
        ('旁边没有东西', [], None),
        ('邻车在侧、间隙 0.325 m', [(0.0, 1.25)], 0.325),
        ('邻车在侧、间隙 0.10 m', [(0.0, 1.025)], 0.10),
        ('已剐上（间隙 0）', [(0.0, 0.925)], 0.0),
        ('车身中段偏前的邻车', [(1.5, 1.25)], 0.325),
        # 车位后墙：纵向 2.5 m → 距车头 0.325 m，**不是** 0（不能误触发）
        ('车位后墙（纵向 2.5 m）', [(2.5, 0.0)], 0.325),
        # 正前方 2.0 m 的墙：落在车体矩形内 → 0（这一条正是旧判据算出 -0.92 的地方）
        ('正前方 2.0 m 的墙', [(2.0, 0.0)], 0.0),
        ('正前方 3.0 m 的墙', [(3.0, 0.0)], 0.825),
    ]
    print('\n泊车车体距离（点到车体矩形的距离，车体内为 0）')
    bad = 0
    for name, pts, want in rows:
        gap, at = body_clear(scan_with(pts), AMIN, INC)
        if want is None:
            ok = gap == float('inf')
        else:
            ok = abs(gap - want) <= 0.02
        bad += 0 if ok else 1
        info = 'inf' if gap == float('inf') else f'{gap:.3f} m'
        exp = 'inf' if want is None else f'{want:.3f} m'
        print(f'  [{"OK" if ok else "--"}] {name:<28} {info:<9} 期望 {exp}')
    print(f'通过 {len(rows) - bad}/{len(rows)}')
    return bad


if __name__ == '__main__':
    sys.exit(main())
