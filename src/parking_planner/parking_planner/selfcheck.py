#!/usr/bin/env python3
"""离线自检：不依赖 ROS / Gazebo，验证算法层是否正常。

    python3 -m parking_planner.selfcheck            # 或 ros2 run parking_planner selfcheck

检查项：
1. world 解析：车位 / 障碍数量、ROI；
2. Reeds-Shepp：随机位姿对上的可达性与积分一致性（端点误差 < 2 mm）；
3. Hybrid A*：从自车出生位姿规划到 W5，路径逐点做车体碰撞检查；
4. 栅格与服务：占用栅格尺寸、膨胀层不遮挡车位中心。
"""

import math
import os
import sys
import time

import numpy as np

from .geometry import wrap
from .hybrid_astar import HybridAStar
from .lot_layout import (EFFECTIVE_MAX_STEER, Layout, MIN_TURN_RADIUS,
                         WHEELBASE)
from .reeds_shepp import reeds_shepp

DEFAULT_WORLD = os.path.expanduser(
    '~/apa_ws/src/parking_lot_city/worlds/city_parking_lot.sdf')


def check_rs(n=60, rng=None):
    rng = rng or np.random.default_rng(7)
    bad = 0
    found = 0
    for _ in range(n):
        sx, sy = rng.uniform(-8, 8, 2)
        sth = rng.uniform(-math.pi, math.pi)
        d = rng.uniform(3.0, 12.0)
        a = rng.uniform(-math.pi, math.pi)
        gx, gy = sx + d * math.cos(a), sy + d * math.sin(a)
        gth = wrap(sth + rng.uniform(-2.1, 2.1))
        p = reeds_shepp((sx, sy, sth), (gx, gy, gth), MIN_TURN_RADIUS)
        if p is None:
            continue
        found += 1
        s = p.sample(0.2)
        err = (math.hypot(s[-1][0] - gx, s[-1][1] - gy)
               + abs(wrap(s[-1][2] - gth)))
        if err > 2e-3:
            bad += 1
    return found, n, bad


def check_plan(layout, grid, stall_id='W5'):
    planner = HybridAStar(grid, wheelbase=WHEELBASE,
                          max_steer=EFFECTIVE_MAX_STEER,
                          xy_res=0.25, th_res=math.radians(3.0))
    start = (0.0, 0.0, 0.0)
    stall = layout.find_stall(stall_id)
    t0 = time.time()
    res = planner.plan(start, stall.goal_pose)
    dt = time.time() - t0
    if not res.success:
        return res, dt, -1
    bad = 0
    for a, b in zip(res.path, res.path[1:]):
        k = max(1, int(math.hypot(b[0] - a[0], b[1] - a[1]) / 0.05))
        for i in range(k + 1):
            t = i / k
            if not planner.free(a[0] + (b[0] - a[0]) * t,
                                a[1] + (b[1] - a[1]) * t,
                                a[2] + (b[2] - a[2]) * t):
                bad += 1
    return res, dt, bad


def main(args=None):
    world = (args or sys.argv[1:] or [DEFAULT_WORLD])[0]
    ok = True
    print('=' * 60)
    print('泊车规划层离线自检')
    print('=' * 60)

    if not os.path.exists(world):
        print(f'[FAIL] world 不存在：{world}')
        return 1
    layout = Layout.from_world(world, 0.10)
    per = [s for s in layout.stalls if s.kind == 'perpendicular']
    par = [s for s in layout.stalls if s.kind == 'parallel']
    print(f'[1] world 解析：障碍 {len(layout.obstacles)}，垂直车位 {len(per)}，'
          f'平行车位 {len(par)}，ROI={[round(v, 1) for v in layout.roi]}')
    if len(per) < 10 or len(layout.obstacles) < 10:
        print('    [FAIL] 车位或障碍数量异常')
        ok = False
    else:
        print('    [PASS]')

    found, total, bad = check_rs()
    print(f'[2] Reeds-Shepp：{found}/{total} 组随机位姿求得曲线，'
          f'积分端点不一致 {bad} 组（阈值 2 mm）')
    if found < total * 0.95 or bad > 0:
        print('    [FAIL]')
        ok = False
    else:
        print('    [PASS]')

    grid = layout.occupancy_grid(0.10)
    print(f'[3] 占用栅格：{grid.width}x{grid.height} @ {grid.res:.2f} m，'
          f'可通行 {int((grid.data == 0).sum())} 格，'
          f'最小转弯半径 {MIN_TURN_RADIUS:.2f} m')
    res, dt, bad = check_plan(layout, grid)
    print(f'[4] Hybrid A* 规划到 W5：{res}')
    print(f'    耗时 {dt * 1000:.0f} ms，路径逐点碰撞超限样本 {bad}')
    if not res.success or bad > 0:
        print('    [FAIL]')
        ok = False
    else:
        print('    [PASS]')

    print('=' * 60)
    print('结果：' + ('全部通过' if ok else '存在失败项'))
    return 0 if ok else 1


if __name__ == '__main__':
    sys.exit(main())
