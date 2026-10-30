#!/usr/bin/env python3
"""E2 预备位姿全维扫描（规划 + 跟踪）：steer × along × side_y，
每个候选做双向交接偏差的闭环仿真，找「低曲率 + 双向稳 + 间隙大」的位姿。

背景（§6.25）：E2 现行 staging（开过车位 3 m 的南侧位姿）在任何
plan_max_steer 下都没有可行解 —— 0.44 是满曲率弧（反馈余量 10%，
换挡瞬态偏差单调发散）；0.42/0.40 弧变大后车尾剐邻车或退成 3 换挡。
而「车位北侧」（没开过车位、提前斜倒）的位姿在 0.40 下存在
1 换挡、κ=0.157 的干净解 —— 本脚本验证它在跟踪层是否真的稳。

坐标系：goal=(62,-32.9,180°)（SPAWN=(-45,15.65) 帧），
side_y>0 = staging 在 goal 南侧（现行），side_y<0 = 北侧（新候选）。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from parking_planner.hybrid_astar import HybridAStar        # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE     # noqa: E402
from verify_park_sim import (CLEARANCE, GOAL_XY_TOL,         # noqa: E402
                             GOAL_YAW_TOL, PARK_K_RATE,
                             PARK_MAX_STEER, PARK_PREVIEW,
                             PARK_STEER_GAIN, PARK_V_MAX,
                             STEER_LAG, TH_RES, UNDER, XY_RES,
                             simulate, traj_gap)
from parking_planner.geometry import wrap                    # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)
APPROACH = -math.pi / 2          # 过道南行


def path_kappa(path, q=0.95):
    ks = []
    for a, b in zip(path, path[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 1e-6 or a[3] != b[3]:
            continue
        ks.append(abs(wrap(b[2] - a[2])) / d)
    return sorted(ks)[min(len(ks) - 1, int(q * len(ks)))] if ks else 0.0


def main():
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    st = layout.find_stall('E2')
    gx, gy, gyaw = st.goal_pose
    ux, uy = math.cos(gyaw), math.sin(gyaw)     # 车位轴（西，指向过道）

    grid = layout.occupancy_grid(XY_RES / 2.0,
                                 roi=(gx - 26, gx + 26, gy - 26, gy + 26))
    print(f'{"steer":6s} {"along":6s} {"sideY":6s} {"换挡":4s} {"长":6s} '
          f'{"余量":6s} {"κ95":7s} {"peak+":7s} {"peak-":7s} '
          f'{"gap+":7s} {"gap-":7s} 判定')
    for steer in (0.40, 0.42, 0.44):
        planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=steer,
                              xy_res=XY_RES, th_res=TH_RES,
                              clearance=CLEARANCE, max_cusps=2)
        for along in (7.0, 8.0, 9.0, 10.0):
            for side_y in (-4.0, -3.0, -2.0, -1.0, 1.0, 2.0, 3.0, 4.0):
                sx = gx + along * ux
                sy = gy - side_y            # side_y>0 → 南（现行方向）
                syaw = APPROACH
                if not planner.free(sx, sy, syaw):
                    continue
                res = planner.plan((sx, sy, syaw), (gx, gy, gyaw))
                if not res.success or res.n_switch > 1:
                    continue
                c = planner.min_clearance(list(res.path))
                if c < 0.30:
                    continue
                k = path_kappa(list(res.path))
                # 双向交接偏差的闭环跟踪（锚定语义：路径起点 = staging）
                peaks, gaps, oks = [], [], []
                for lat in (0.2, -0.2):
                    hand = (sx - math.sin(syaw) * lat,
                            sy + math.cos(syaw) * lat, syaw)
                    final, t, mx, mxt, sh, cmd, traj = simulate(
                        list(res.path), hand)
                    e = math.hypot(final[0] - gx, final[1] - gy)
                    ey = math.degrees(abs(wrap(final[2] - gyaw)))
                    gap, _ = traj_gap(traj, layout.obstacles)
                    peaks.append(max(mx, mxt))
                    gaps.append(gap)
                    oks.append(e <= 0.35 and ey <= 9.0 and gap > 0.0
                               and not cmd.stuck)
                ok = 'OK ' if all(oks) else 'BAD'
                print(f'{steer:6.2f} {along:6.1f} {side_y:6.1f} '
                      f'{res.n_switch:4d} {res.length:6.1f} {c:+6.2f} '
                      f'{k:7.4f} {peaks[0]:7.2f} {peaks[1]:7.2f} '
                      f'{gaps[0]:+7.2f} {gaps[1]:+7.2f} {ok}')


if __name__ == '__main__':
    main()
