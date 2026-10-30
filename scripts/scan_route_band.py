#!/usr/bin/env python3
"""预备位姿「路线可达带」细扫（§6.30 后续）。

scan_dense_staging.py 的盲区：只检查了「staging→goal 可规划」，没检查「路线段
能不能延伸到 staging」。verify_park_sim 的 DIAG_STAGE 实测：
  W1 (5,+4)  hand 距 staging 5.15 m（锚定失效，直连 2 换挡 25.9 m）
  E0 (5,-2.5) hand 距 staging 4.07 m（direct 从过道节点出发，搜索耗尽）
根因：RouteGraph.plan 的终点是**图节点**，staging 距最近节点 1.6~12 m 时只有
「末段行进方向 → staging」夹角 <55° 才延伸；handoff 的截断/补线只处理法向
偏差 ≤0.6 m 的情形。所以 staging 必须落在**过道边**（±0.6 m）上。

W/E 排的过道是南北向（world x=±8，帧内 x=37/53），staging 沿车位轴 along≈9
时正好压在过道线上（W1 goal_x=28 → 28+9=37；E0 goal_x=62 → 62-9=53）。
本脚本沿这条带细扫（along 步长 0.25、lat 步长 0.25），补上稠密扫描
（步长 0.5/1.0）漏掉的可行点。
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
# 沿过道线 ±0.6 m（handoff 法向容差）细扫；along 基准 9（压在过道中心线上）
ALONGS = [9.0 + 0.25 * i for i in range(-2, 3)]
LATS = [round(-12.0 + 0.25 * i, 2) for i in range(97)]


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


def edge_gap(graph, x, y):
    """点到最近图边的距离（路线可达性的粗筛：handoff 法向容差 0.6 m）。"""
    best = float('inf')
    for a, b in graph.edges:
        ax, ay = graph.nodes[a]
        bx, by = graph.nodes[b]
        dx, dy = bx - ax, by - ay
        L2 = dx * dx + dy * dy
        if L2 < 1e-9:
            d = math.hypot(x - ax, y - ay)
        else:
            t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / L2))
            d = math.hypot(x - (ax + t * dx), y - (ay + t * dy))
        best = min(best, d)
    return best


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
                    eg = edge_gap(graph, cx_, cy_)
                    if eg > 0.6:
                        continue
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
                    rows.append((along, lat, r.n_switch, round(r.length, 1),
                                 round(eg, 2),
                                 round(planner.body_clearance(path), 3),
                                 round(planner.min_clearance(path), 3),
                                 round(kappa_q(path), 4), n_flips(path)))
        finally:
            planner.max_iter = saved
        rows.sort(key=lambda t: (t[2], -t[5], t[3]))
        print(f'\n=== {tid} goal=({gx:.1f},{gy:.1f}) 候选朝向 '
              f'{math.degrees(syaw):.0f}deg（过道带细扫）===')
        print(f'{"along":>6s} {"lat":>6s} {"换挡":>4s} {"长度":>6s} '
              f'{"边距":>5s} {"bodyC":>6s} {"circC":>6s} {"κ95":>7s} {"flip":>4s}')
        for t in rows[:30]:
            print(f'{t[0]:6.2f} {t[1]:+6.2f} {t[2]:4d} {t[3]:6.1f} '
                  f'{t[4]:5.2f} {t[5]:+6.3f} {t[6]:+6.3f} {t[7]:7.4f} '
                  f'{t[8]:4d}')
        print(f'共 {len(rows)} 条可行解（max_iter=1 解析解）', flush=True)


if __name__ == '__main__':
    main()
