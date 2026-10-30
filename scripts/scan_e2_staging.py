#!/usr/bin/env python3
"""E2 预备位姿几何扫描：枚举 along × side × 曲率上限，评价每条直连解的
换挡数 / 规划余量 / 路径曲率，为「低曲率 + 大余量」的预备位姿提供定量依据。

背景（真机 a3 失败链，见 docs/06 §6.24）：
  E2 现行 staging (along=9, side=3) 的直连解是满曲率弧 κ=0.175，
  占执行限幅 0.202 的 86% —— 换挡瞬态 ~0.16 m 偏差就让指令顶死、
  偏差单调发散（离线 HANDOFF_LAT=-0.2 完全复现）。
  微调 plan_max_steer（0.42）只对 yaw=0 的残差有效，yaw=+4° 又全挂，
  说明要换几何而不是换参数。

输出：按「换挡数 → 规划余量 → 曲率」排序的候选表。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import wrap                        # noqa: E402
from parking_planner.hybrid_astar import HybridAStar             # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE         # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)
XY_RES, TH_RES = 0.25, math.radians(3.0)
CLEARANCE = 0.15          # plan_clearance 默认值（与 planner_node 一致）


def path_kappa(path, q=0.95):
    """路径差分曲率的 q 分位（避开换挡点/端点的奇异差分）。"""
    ks = []
    for a, b in zip(path, path[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 1e-6 or a[3] != b[3]:      # 零长段 / 换挡点跳过
            continue
        ks.append(abs(wrap(b[2] - a[2])) / d)
    if not ks:
        return 0.0
    ks.sort()
    return ks[min(len(ks) - 1, int(q * len(ks)))]


def main():
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    st = layout.find_stall('E2')
    gx, gy, gyaw = st.goal_pose
    ux, uy = math.cos(gyaw), math.sin(gyaw)     # 车位轴（指向过道）
    nx, ny = -uy, ux                             # 左法向
    approach = -math.pi / 2                      # 过道南行

    grid = layout.occupancy_grid(XY_RES / 2.0,
                                 roi=(gx - 16, gx + 16, gy - 16, gy + 16))
    rows = []
    for steer in (0.44, 0.42, 0.40):
        planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=steer,
                              xy_res=XY_RES, th_res=TH_RES,
                              clearance=CLEARANCE, max_cusps=2)
        saved = planner.max_iter
        planner.max_iter = 1                     # 只求 RS 解析解
        try:
            for along in (7.0, 7.5, 8.0, 8.5, 9.0, 9.5, 10.0, 10.5, 11.0):
                for side in (1.5, 2.0, 2.5, 3.0, 3.5, 4.0, 4.5):
                    sx = gx + along * ux - side * nx   # side 沿过道正向偏移
                    sy = gy + along * uy - side * ny
                    syaw = approach
                    if not planner.free(sx, sy, syaw):
                        continue
                    r = planner.plan((sx, sy, syaw), (gx, gy, gyaw))
                    if not r.success:
                        continue
                    c = planner.min_clearance(list(r.path))
                    k = path_kappa(list(r.path))
                    rows.append((steer, along, side, r.n_switch,
                                 round(r.length, 1), round(c, 2), round(k, 4)))
        finally:
            planner.max_iter = saved

    # 排序：换挡少 → 余量大 → 曲率低
    rows.sort(key=lambda t: (t[3], -t[5], t[6]))
    print(f'{"steer":6s} {"along":6s} {"side":5s} {"换挡":4s} '
          f'{"长度":7s} {"余量":7s} {"κ95":7s}')
    for t in rows[:40]:
        print(f'{t[0]:6.2f} {t[1]:6.1f} {t[2]:5.1f} {t[3]:4d} '
              f'{t[4]:7.1f} {t[5]:+7.2f} {t[6]:7.4f}')
    print(f'\n共 {len(rows)} 条可行解（当前线上值 along=9 side=3 steer=0.44）')

    cur = [t for t in rows if abs(t[1] - 9.0) < 1e-6 and abs(t[2] - 3.0) < 1e-6
           and abs(t[0] - 0.44) < 1e-6]
    if cur:
        t = cur[0]
        print(f'现行配置：换挡 {t[3]}，余量 {t[5]:+.2f}，κ95={t[6]:.4f}，'
              f'κ/限幅={t[6]/0.2019:.0%}')


if __name__ == '__main__':
    main()
