#!/usr/bin/env python3
"""对比「直接解」与「两段式」在同一交接位姿下的**计划余量**与**执行后真实间隙**。

用途：判断 `_plan` 的选解规则（现在按 (换挡次数, 长度) 字典序，0 换挡永远优先）
是否应该改成「余量优先」。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import body_gap                            # noqa: E402
from parking_planner.hybrid_astar import HybridAStar                     # noqa: E402
from parking_planner.lot_layout import (EFFECTIVE_MAX_STEER, Layout,      # noqa: E402
                                        WHEELBASE)
from parking_planner.planner_node import sample_straight                 # noqa: E402
from parking_planner.route_planner import (RouteGraph,                   # noqa: E402
                                           default_graph_path)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from verify_park_sim import (CLEARANCE, SPAWN, TH_RES, WORLD, XY_RES,     # noqa: E402
                             handoff_pose, simulate, traj_gap)


def main():
    tid = sys.argv[1] if len(sys.argv) > 1 else 'W7'
    back = float(sys.argv[2]) if len(sys.argv) > 2 else 0.25
    lat = float(sys.argv[3]) if len(sys.argv) > 3 else 0.25
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
    st = layout.find_stall(tid)
    route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
    hand = handoff_pose(route, back, lat)
    pts = [hand, st.goal_pose, st.staging_pose]
    cx = sum(p[0] for p in pts) / 3.0
    cy = sum(p[1] for p in pts) / 3.0
    half = max(15.0, max(math.hypot(p[0] - cx, p[1] - cy) for p in pts) + 6.0)
    grid = layout.occupancy_grid(XY_RES / 2.0,
                                 roi=(cx - half, cx + half, cy - half, cy + half))
    pl = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=EFFECTIVE_MAX_STEER,
                     xy_res=XY_RES, th_res=TH_RES, clearance=CLEARANCE, max_cusps=2)
    print(f'{tid}  交接位姿：回退 {back} m + 横向 {lat:+.2f} m')
    print(f'  交接位姿 = ({hand[0]:.2f},{hand[1]:.2f},{math.degrees(hand[2]):.1f}°)  '
          f'预备位姿 = ({st.staging_pose[0]:.2f},{st.staging_pose[1]:.2f},'
          f'{math.degrees(st.staging_pose[2]):.1f}°)')

    cands = []
    d = pl.plan(hand, st.goal_pose)
    if d.success:
        cands.append(('直接', d, list(d.path)))
    r = pl.plan(hand, st.staging_pose)
    if r.success:
        p2 = list(r.path) + sample_straight(st.staging_pose, st.goal_pose, 0.10)
        cands.append(('两段式', r, p2))

    print(f'\n  {"解":6s} {"换挡":4s} {"长度":7s} {"计划余量":9s} '
          f'{"执行最小间隙":12s} {"终点误差":8s}')
    for name, res, path in cands:
        plan_gap = pl.min_clearance(path) if hasattr(pl, 'min_clearance') else None
        final, t, maxlat, shifts, cmd, traj = simulate(path, hand)
        gap, _ = traj_gap(traj, layout.obstacles)
        e = math.hypot(final[0] - st.goal_pose[0], final[1] - st.goal_pose[1])
        pg = f'{plan_gap:+.3f} m' if plan_gap is not None else '-'
        print(f'  {name:6s} {res.n_switch:4d} {res.length:7.1f} {pg:9s} '
              f'{gap:+11.2f} m {e:8.2f}')


if __name__ == '__main__':
    main()
