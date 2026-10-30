#!/usr/bin/env python3
"""离线验证：路线终点 → Hybrid A* → 车位，覆盖场内垂直/场内平行/路侧平行。"""
import math
import sys
import time

sys.path.insert(0, os.path.expanduser("~/apa_ws/src/parking_planner"))

from parking_planner.hybrid_astar import HybridAStar          # noqa: E402
from parking_planner.lot_layout import (EFFECTIVE_MAX_STEER, Layout,  # noqa: E402
                                        WHEELBASE)
from parking_planner.route_planner import RouteGraph, default_graph_path  # noqa: E402
import os

WORLD = os.path.expanduser("~/apa_ws/src/parking_lot_city/worlds/city_parking_lot.sdf")
SPAWN = (-45.0, 15.65, 0.0)
XY_RES, TH_RES = 0.25, math.radians(3.0)


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def main():
    raw = Layout.from_world(WORLD)
    layout = raw.to_frame(SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
    ids = ["W1", "W5", "W9", "E0", "E6", "P1", "P3",
           "SW1", "SW2", "SW4", "SE1", "SE4", "B1", "B3", "B5"]
    for tid in ids:
        st = layout.find_stall(tid)
        staging = st.staging_pose
        route = graph.plan((0.0, 0.0), staging, start_heading=0.0)
        if not route:
            print(f"{tid:4s} 路线规划失败")
            continue
        rlen = sum(dist(a, b) for a, b in zip(route, route[1:]))
        # 交接位姿 = 路线终点（路线走完才切 Hybrid A*，与 planner_node 一致）
        hand = route[-1]
        d_stage = dist(hand, staging)
        # 建局部窗口（与 planner_node 一致）
        pts = [hand, st.goal_pose, staging]
        cx = sum(p[0] for p in pts) / 3.0
        cy = sum(p[1] for p in pts) / 3.0
        half = max(18.0, max(dist(p, (cx, cy)) for p in pts) + 6.0)
        roi = (cx - half, cx + half, cy - half, cy + half)
        grid = layout.occupancy_grid(XY_RES / 2.0, roi=roi)
        planner = HybridAStar(grid, wheelbase=WHEELBASE,
                              max_steer=EFFECTIVE_MAX_STEER, xy_res=XY_RES,
                              th_res=TH_RES,
                              max_cusps=3 if st.kind == "parallel" else 2)
        t0 = time.time()
        if st.kind == "parallel":
            a = planner.plan(hand, staging)
            b = planner.plan(staging, st.goal_pose) if a.success else None
            if b is not None and b.success:
                res = a
                res.success = True
                res.length = a.length + b.length
                res.reverse_length = a.reverse_length + b.reverse_length
                res.n_switch = a.n_switch + b.n_switch
                res.expansions = a.expansions + b.expansions
                res.reason = "两段式"
            else:
                res = planner.plan(hand, st.goal_pose)
        else:
            res = planner.plan(hand, st.goal_pose)
        dt = time.time() - t0
        flag = "OK " if res.success else "FAIL"
        print(f"{flag} {tid:4s} {st.kind:13s}/{st.region:10s} 路线 {rlen:6.1f} m "
              f"交接距预备位姿 {d_stage:4.1f} m | 局部 "
              f"{'成功' if res.success else res.reason} 长度 {res.length:5.1f} m "
              f"倒车 {res.reverse_length:5.1f} m 换挡 {res.n_switch} "
              f"用时 {dt:4.2f}s 窗口 {2 * half:.0f} m")


if __name__ == "__main__":
    main()
