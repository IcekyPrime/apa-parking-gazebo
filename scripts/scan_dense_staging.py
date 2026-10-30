#!/usr/bin/env python3
"""预备位姿稠密扫描（§6.30 修正换算 + §6.29 后轴语义，三口径余量）。

与 scan_fix_staging.py 的区别：
1. 网格补上 lat ∈ (-5, +5) 盲区 —— E2 的成功条目就在 lat=-3，上一轮网格
   （只扫 ±5 以外）对 E0 给出「仅 lat=-7 可行」的结论可能只是没扫到；
2. 余量同时报 body_gap（几何精确、真机预测口径，W7 用它预测 0.44 实测吻合）、
   body_clearance（栅格 EDT+轮廓）、min_clearance（7 圆模型）三个口径 ——
   上一轮只报 min_clearance，文档明确警告圆模型在车角处保守 ~0.1 m。
排序：换挡少 → body_gap 大 → 长度短。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS',
                                      os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import body_gap           # noqa: E402
from parking_planner.hybrid_astar import HybridAStar    # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE  # noqa: E402
from parking_planner.route_planner import (RouteGraph,  # noqa: E402
                                           default_graph_path)

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
SPAWN = (-30.0, 15.65, 0.0)
XY_RES, TH_RES = 0.25, math.radians(3.0)
WRAP = lambda a: (a + math.pi) % (2 * math.pi) - math.pi   # noqa: E731

STALLS = sys.argv[1].split(',') if len(sys.argv) > 1 else ['E0', 'W1']
DYAW = {'W': -math.pi / 2, 'E': math.pi / 2}
ALONGS = [5.0, 6.0, 7.0, 8.0, 9.0, 10.0, 11.0, 12.0]
LATS = [round(-8.0 + 0.5 * i, 2) for i in range(33)]


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
                    cx_ = gx + along * ux + lat * nx
                    cy_ = gy + along * uy + lat * ny
                    if not graph.plan((0.0, 0.0), (cx_, cy_, syaw),
                                      start_heading=0.0):
                        continue
                    fx = cx_ - math.cos(syaw) * ax
                    fy = cy_ - math.sin(syaw) * ax
                    if not planner.free(fx, fy, syaw):
                        continue
                    r = planner.plan((fx, fy, syaw), (gx_r, gy_r, gyaw))
                    if not r.success:
                        continue
                    path = list(r.path)
                    bg = min(body_gap((p[0], p[1], p[2]), layout.obstacles)
                             for p in path)
                    rows.append((along, lat, r.n_switch, round(r.length, 1),
                                 round(bg, 3),
                                 round(planner.body_clearance(path), 3),
                                 round(planner.min_clearance(path), 3),
                                 round(kappa_q(path), 4), n_flips(path)))
        finally:
            planner.max_iter = saved
        rows.sort(key=lambda t: (t[2], -t[4], t[3]))
        print(f'\n=== {tid} goal=({gx:.1f},{gy:.1f}) '
              f'候选朝向 {math.degrees(syaw):.0f}deg（稠密网格/三口径）===')
        print(f'{"along":>6s} {"lat":>6s} {"换挡":>4s} {"长度":>6s} '
              f'{"bodyG":>6s} {"bodyC":>6s} {"circC":>6s} {"κ95":>7s} '
              f'{"flip":>4s}')
        for t in rows[:25]:
            print(f'{t[0]:6.1f} {t[1]:+6.2f} {t[2]:4d} {t[3]:6.1f} '
                  f'{t[4]:+6.3f} {t[5]:+6.3f} {t[6]:+6.3f} {t[7]:7.4f} '
                  f'{t[8]:4d}')
        print(f'共 {len(rows)} 条可行解（max_iter=1 解析解）', flush=True)


if __name__ == '__main__':
    main()
