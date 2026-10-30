"""RS 候选解余量扫描：对每个车位枚举全部模式族的解析解，按余量对比。

回答：W5/W7/E3/E8 的 0.068 m 最短解之外，是否存在更安全的同起终点解析解？
"""
import math
import os
import sys

WS = os.path.expanduser('~/apa_ws')
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from parking_planner.hybrid_astar import HybridAStar
from parking_planner.lot_layout import Layout, WHEELBASE
from parking_planner.route_planner import RouteGraph, default_graph_path
from parking_planner.reeds_shepp import reeds_shepp
from parking_planner.geometry import body_gap

import verify_park_sim as V

STALLS = sys.argv[1].split(',') if len(sys.argv) > 1 else \
    ['W1', 'W4', 'W5', 'W7', 'W9', 'E0', 'E2', 'E3', 'E6', 'E7', 'E8']

layout = Layout.from_world(V.WORLD).to_frame(V.SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(V.SPAWN)

for tid in STALLS:
    st = layout.find_stall(tid)
    route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
    hand = V.handoff_pose(route, 0.0, 0.0, 0.0, target=st.staging_pose)
    pts = [hand, st.goal_pose, st.staging_pose]
    cx = sum(p[0] for p in pts) / 3.0
    cy = sum(p[1] for p in pts) / 3.0
    half = 25.0
    grid = layout.occupancy_grid(V.XY_RES / 2.0,
                                 roi=(cx - half, cx + half, cy - half, cy + half))
    planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=V.PLAN_MAX_STEER,
                          xy_res=V.XY_RES, th_res=V.TH_RES,
                          clearance=V.CLEARANCE, max_cusps=2)
    radius = WHEELBASE / math.tan(V.PLAN_MAX_STEER)
    cands = reeds_shepp(hand, st.goal_pose, radius, max_cusps=2,
                        quick=False, all_candidates=True)
    print(f'\n=== {tid}: {len(cands)} 个模式族解（hand→goal 直连） ===')
    print(f'{"pattern":28s} {"len":>6s} {"cusp":>4s} {"circle_clr":>10s} '
          f'{"body_gap":>8s} {"碰撞":>4s}')
    for rs in cands:
        samples = rs.sample(0.20)
        if not samples:
            continue
        collide = False
        for (x, y, th, _d, _c) in samples:
            if math.hypot(x - hand[0], y - hand[1]) <= planner.start_free_radius:
                continue
            if not planner.free(x, y, th):
                collide = True
                break
        clr = planner.min_clearance(samples) if not collide else float('nan')
        bg = min(body_gap((p[0], p[1], p[2]), layout.obstacles)
                 for p in samples) if not collide else float('nan')
        pat = '+'.join(f'{k}{"+" if s > 0 else ("-0" if s == 0 else "-")}'
                       for k, s in rs.pattern)
        print(f'{pat:28s} {rs.length:6.2f} {rs.n_cusps:4d} '
              f'{clr:10.3f} {bg:+8.3f} {"YES" if collide else "":>4s}')
