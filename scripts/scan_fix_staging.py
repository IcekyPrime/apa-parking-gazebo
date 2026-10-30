#!/usr/bin/env python3
"""预备位姿宽网格扫描（§6.30 修正换算版）。

与 scan_staging_gap.py 的区别：候选/终点都按 §6.29 后轴语义正确换算
（候选沿**自身朝向**后移轴距/2，而不是沿车位轴 —— 那个偏差 ~1.9 m 的
bug 让 W1/E0 的在线搜索全军覆没）。输出按「换挡少 → 余量大 → 曲率小」
排序的前 20 条，供写 config/staging_poses.yaml 条目用。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.hybrid_astar import HybridAStar   # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE  # noqa: E402
from parking_planner.route_planner import (RouteGraph,  # noqa: E402
                                           default_graph_path)

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
SPAWN = (-30.0, 15.65, 0.0)
XY_RES, TH_RES = 0.25, math.radians(3.0)
WRAP = lambda a: (a + math.pi) % (2 * math.pi) - math.pi   # noqa: E731

STALLS = sys.argv[1].split(',') if len(sys.argv) > 1 else ['W1', 'E0']
DYAW = {'W': -math.pi / 2, 'E': math.pi / 2}
# 网格：along × |lat|（lat 带符号时 side=±1 都扫）
ALONGS = [7.0, 8.0, 9.0, 10.0, 11.0]
LATS = [-8.0, -7.0, -6.0, -5.735, -5.0, 5.0, 5.735, 6.0, 7.0, 8.0]


def kappa_q(path, q=0.95):
    ks = []
    for a, b in zip(path, path[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 1e-6 or a[3] != b[3]:
            continue
        ks.append(abs(WRAP(b[2] - a[2])) / d)
    if not ks:
        return 0.0
    ks.sort()
    return ks[min(len(ks) - 1, int(q * len(ks)))]


def n_flips(path):
    n = 0
    for a, b in zip(path, path[1:]):
        if a[3] == b[3] and a[4] * b[4] < -1e-12:
            n += 1
    return n


def main():
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
    ax = WHEELBASE * 0.5
    for tid in STALLS:
        st = layout.find_stall(tid)
        gx, gy, gyaw = st.goal_pose
        ux, uy = math.cos(gyaw), math.sin(gyaw)
        nx, ny = -uy, ux
        gx_r, gy_r = gx - ux * ax, gy - uy * ax
        syaw = WRAP(gyaw + DYAW[tid[0]])
        grid = layout.occupancy_grid(
            XY_RES / 2.0, roi=(gx - 20, gx + 20, gy - 20, gy + 20))
        planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=0.42,
                              xy_res=XY_RES, th_res=TH_RES, clearance=0.15,
                              max_cusps=2, clear_target=0.30)
        saved = planner.max_iter
        planner.max_iter = 1
        rows = []
        try:
            for along in ALONGS:
                for lat in LATS:
                    cx_ = gx + along * ux + lat * nx   # 中心语义候选
                    cy_ = gy + along * uy + lat * ny
                    # 路线可达（中心语义，与 planner_node._extend_to_staging 同判）
                    if not graph.plan((0.0, 0.0), (cx_, cy_, syaw),
                                      start_heading=0.0):
                        continue
                    fx = cx_ - math.cos(syaw) * ax     # 候选中心 → 后轴
                    fy = cy_ - math.sin(syaw) * ax
                    if not planner.free(fx, fy, syaw):
                        continue
                    r = planner.plan((fx, fy, syaw), (gx_r, gy_r, gyaw))
                    if not r.success:
                        continue
                    clr = planner.min_clearance(list(r.path))
                    rows.append((along, lat, r.n_switch, round(r.length, 1),
                                 round(clr, 3),
                                 round(kappa_q(list(r.path)), 4),
                                 n_flips(list(r.path))))
        finally:
            planner.max_iter = saved
        rows.sort(key=lambda t: (t[2], -t[4], t[5]))
        print(f'\n=== {tid} goal=({gx:.1f},{gy:.1f}) '
              f'候选朝向 {math.degrees(syaw):.0f}deg（修正换算）===')
        print(f'{"along":>6s} {"lat":>6s} {"换挡":>4s} {"长度":>6s} '
              f'{"余量":>6s} {"κ95":>7s} {"flip":>4s}')
        for t in rows[:20]:
            print(f'{t[0]:6.1f} {t[1]:+6.2f} {t[2]:4d} {t[3]:6.1f} '
                  f'{t[4]:+6.3f} {t[5]:7.4f} {t[6]:4d}')
        print(f'共 {len(rows)} 条可行解（max_iter=1 解析解）')


if __name__ == '__main__':
    main()
