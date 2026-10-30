#!/usr/bin/env python3
"""E2 规划链路复现：hand 位姿、plan 结构、strip_micro_head 内部量。"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))
from verify_park_sim import (CLEARANCE, HANDOFF_BACK, HANDOFF_LAT,  # noqa
                             HANDOFF_YAW, PLAN_AT_STAGE, PLAN_MAX_STEER, SPAWN,
                             WORLD, XY_RES, TH_RES, handoff_pose, plan_path,
                             dist)
from parking_planner.geometry import wrap                            # noqa: E402
from parking_planner.hybrid_astar import HybridAStar                 # noqa: E402
from parking_planner.lot_layout import WHEELBASE, Layout             # noqa: E402
from parking_planner.route_planner import (RouteGraph,               # noqa: E402
                                           default_graph_path)


def main():
    tid = sys.argv[1] if len(sys.argv) > 1 else 'E2'
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
    st = layout.find_stall(tid)
    print(f'staging(map) = {tuple(round(v, 3) for v in st.staging_pose)}')
    print(f'goal(map)    = {tuple(round(v, 3) for v in st.goal_pose)}')

    route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
    print(f'route: {"ok" if route else "FAIL"}，末 3 点 '
          f'{[tuple(round(v, 2) for v in p) for p in route[-3:]]}')

    hand = handoff_pose(route, HANDOFF_BACK, HANDOFF_LAT, HANDOFF_YAW,
                        target=st.staging_pose)
    print(f'hand = {tuple(round(v, 3) for v in hand)}  '
          f'（距 staging {dist(hand, st.staging_pose):.3f} m）')

    gx, gy, gyaw = st.goal_pose
    cx = (hand[0] + gx) / 2.0
    cy = (hand[1] + gy) / 2.0
    half = max(15.0, max(dist(p, (cx, cy)) for p in (hand, st.goal_pose)) + 6.0)
    grid = layout.occupancy_grid(XY_RES / 2.0,
                                 roi=(cx - half, cx + half, cy - half, cy + half))
    planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=PLAN_MAX_STEER,
                          xy_res=XY_RES, th_res=TH_RES, clearance=CLEARANCE,
                          max_cusps=2)

    raw = planner.plan(hand, st.goal_pose)
    print(f'\nraw plan: 成功={raw.success} 长度={raw.length:.2f} '
          f'换挡={raw.n_switch} 原因={raw.reason}')
    path = list(raw.path)
    i = next((k for k in range(1, len(path)) if path[k][3] != path[0][3]), None)
    print(f'首点方向={path[0][3]}, 首换挡下标 i={i}')
    if i is not None:
        leg = sum(dist(a, b) for a, b in zip(path[:i], path[1:i + 1]))
        print(f'首段长 leg={leg:.3f}（MIN_SEG_LEN=1.2）, cusp 位姿='
              f'{tuple(round(v, 2) for v in path[i][:3])}')
        r = planner.plan(path[i][:3], st.goal_pose)
        print(f'从 cusp 重规划: 成功={r.success} 长度={r.length:.2f} '
              f'换挡={r.n_switch} 原因={r.reason}')
        if r.success:
            c = planner.min_clearance(list(r.path))
            print(f'  重规划弧余量 {c:+.2f}')

    res, path2, used = plan_path(planner, hand, st)
    print(f'\nplan_path 结果: 原因={res.reason} 长度={res.length:.2f} '
          f'换挡={res.n_switch} 两段={used}')
    print(f'hand 距 staging {dist(hand, st.staging_pose):.3f} m，'
          f'route 末点 {tuple(round(v, 2) for v in route[-1])}')
    dirs = [p[3] for p in path2]
    segs = []
    for a, b in zip(path2, path2[1:]):
        ds = math.hypot(b[0] - a[0], b[1] - a[1])
        if segs and segs[-1][0] == b[3]:
            segs[-1][1] += ds
        else:
            segs.append([b[3], ds])
    print('段结构:', ' | '.join(f'{"F" if d > 0 else "R"} {L:.2f} m'
                               for d, L in segs))
    print(f'路径起点 {tuple(round(v, 2) for v in path2[0][:3])}，'
          f'起点-手位姿差 {dist(path2[0], hand):.3f} m / '
          f'{math.degrees(abs(wrap(path2[0][2] - hand[2]))):.1f}°')

    # 曲率剖面：按弧长每 0.5 m 打一点，看倒车弧是否顶在规划上限 0.1732
    print('\n曲率剖面 (s | dir | yaw° | c):')
    s_acc, next_mark = 0.0, 0.0
    for a, b in zip(path2, path2[1:]):
        ds = dist(a, b)
        while next_mark <= s_acc + ds:
            u = (next_mark - s_acc) / ds if ds > 1e-9 else 0.0
            yaw_i = wrap(a[2] + wrap(b[2] - a[2]) * u)
            print(f'  {next_mark:5.2f}  {"F" if b[3] > 0 else "R"}  '
                  f'{math.degrees(yaw_i):7.2f}  {b[4]:+.4f}')
            next_mark += 0.5
        s_acc += ds


if __name__ == '__main__':
    main()
