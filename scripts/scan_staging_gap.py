#!/usr/bin/env python3
"""低余量车位的预备位姿扫描：along × lateral 网格，评每格直连解的
换挡数 / 真实车体余量 / 曲率 / 同档变号数，并验证路线可达。

背景（docs/06 §6.26）：W7/E3 在现行 staging（lateral ±5.735）下，
全部 RS 模式族的直连解要么碰撞、要么余量 ≤0.164 m —— 几何本身没救，
要换预备位姿（E2 的先例：+3 → -3 后余量 0.068 → 0.319）。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import body_gap, wrap          # noqa: E402
from parking_planner.hybrid_astar import HybridAStar         # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE     # noqa: E402
from parking_planner.route_planner import (RouteGraph,        # noqa: E402
                                           default_graph_path)

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)
XY_RES, TH_RES = 0.25, math.radians(3.0)
CLEARANCE = 0.15
STEER = 0.42

STALLS = sys.argv[1].split(',') if len(sys.argv) > 1 else ['W7', 'E3']
# 每个车位保留行当前的 dyaw（W 排 -90°，E 排 +90°）
DYAW = {'W': -math.pi / 2, 'E': math.pi / 2}


def path_kappa(path, q=0.95):
    ks = []
    for a, b in zip(path, path[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 1e-6 or a[3] != b[3]:
            continue
        ks.append(abs(wrap(b[2] - a[2])) / d)
    if not ks:
        return 0.0
    ks.sort()
    return ks[min(len(ks) - 1, int(q * len(ks)))]


def n_flips(path):
    """同档位内曲率变号次数（跟踪难度指标，E2 的收尾弧就是这种flip）。"""
    n = 0
    for a, b in zip(path, path[1:]):
        if a[3] == b[3] and a[4] * b[4] < -1e-12:
            n += 1
    return n


def main():
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
    for tid in STALLS:
        st = layout.find_stall(tid)
        gx, gy, gyaw = st.goal_pose
        c, s = math.cos(gyaw), math.sin(gyaw)
        dyaw = DYAW[tid[0]]
        grid = layout.occupancy_grid(XY_RES / 2.0,
                                     roi=(gx - 20, gx + 20, gy - 20, gy + 20))
        planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=STEER,
                              xy_res=XY_RES, th_res=TH_RES,
                              clearance=CLEARANCE, max_cusps=2,
                              clear_target=0.30)
        saved = planner.max_iter
        planner.max_iter = 1
        rows = []
        try:
            for along in (7.0, 8.0, 9.0, 10.0, 11.0):
                for lat in [v * 0.5 for v in range(-12, 13)]:
                    sx = gx + c * along - s * lat
                    sy = gy + s * along + c * lat
                    syaw = wrap(gyaw + dyaw)
                    if not planner.free(sx, sy, syaw):
                        continue
                    if not graph.plan((0.0, 0.0), (sx, sy, syaw),
                                      start_heading=0.0):
                        continue
                    r = planner.plan((sx, sy, syaw), (gx, gy, gyaw))
                    if not r.success:
                        continue
                    clr = planner.body_clearance(list(r.path))
                    bg = min(body_gap((p[0], p[1], p[2]), layout.obstacles)
                             for p in r.path)
                    rows.append((along, lat, r.n_switch, round(r.length, 1),
                                 round(bg, 3), round(clr, 3),
                                 round(path_kappa(list(r.path)), 4),
                                 n_flips(list(r.path))))
        finally:
            planner.max_iter = saved
        # 排序：真实 body_gap 降序 → 换挡升序 → 长度升序
        rows.sort(key=lambda t: (-t[4], t[2], t[3]))
        print(f'\n=== {tid}（goal=({gx:.1f},{gy:.1f}), dyaw={math.degrees(dyaw):.0f}deg,'
              f' 现行 staging 见 yaml）===')
        print(f'{"along":>6s} {"lat":>5s} {"换挡":>4s} {"长度":>6s} '
              f'{"body":>6s} {"circle":>6s} {"κ95":>7s} {"flip":>4s}')
        for t in rows[:25]:
            print(f'{t[0]:6.1f} {t[1]:5.1f} {t[2]:4d} {t[3]:6.1f} '
                  f'{t[4]:+6.3f} {t[5]:+6.3f} {t[6]:7.4f} {t[7]:4d}')
        print(f'共 {len(rows)} 条可行解')


if __name__ == '__main__':
    main()
