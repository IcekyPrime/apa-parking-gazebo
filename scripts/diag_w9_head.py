#!/usr/bin/env python3
"""W9 起点航向缺陷诊断：t=0 时 φ=−14.3° 到底出在哪一环。

逐项打印：路线末段方向、staging 位姿、handoff 位姿（回退后）、锚定判据、
规划路径前几个采样点的切线、PathFollower 第一拍的投影与 φ/e。
对比三种条件：HANDOFF_*=0、基线(0.3/0.15/0°)、基线+8°。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.hybrid_astar import HybridAStar                    # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE                # noqa: E402
from parking_planner.geometry import wrap                                # noqa: E402
from parking_planner.planner_node import dedup_path, sample_straight     # noqa: E402
from parking_planner.route_planner import RouteGraph, default_graph_path  # noqa: E402

import importlib                                                        # noqa: E402
vps = importlib.import_module('verify_park_sim') if os.path.exists(
    'verify_park_sim.py') else None


def seg_yaw(a, b):
    return math.atan2(b[1] - a[1], b[0] - a[0])


def main():
    layout = Layout.from_world(vps.WORLD).to_frame(vps.SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(vps.SPAWN)
    st = layout.find_stall('W9')
    route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
    print(f'W9 goal   = ({st.goal_pose[0]:+.2f}, {st.goal_pose[1]:+.2f}, '
          f'{math.degrees(st.goal_pose[2]):+.1f}deg)')
    print(f'W9 stage  = ({st.staging_pose[0]:+.2f}, {st.staging_pose[1]:+.2f}, '
          f'{math.degrees(st.staging_pose[2]):+.1f}deg)')
    print(f'route 末 5 点（含对齐段）:')
    for p in route[-5:]:
        print(f'  ({p[0]:+.2f}, {p[1]:+.2f}, {math.degrees(p[2]):+.1f}deg)')
    if len(route) >= 2:
        print(f'末段方向 seg_yaw = '
              f'{math.degrees(seg_yaw(route[-2], route[-1])):+.1f}deg, '
              f'staging_yaw = {math.degrees(st.staging_pose[2]):+.1f}deg, '
              f'差 = {math.degrees(wrap(seg_yaw(route[-2], route[-1]) - st.staging_pose[2])):+.1f}deg')

    for back, lat, dyaw in ((0.0, 0.0, 0.0), (0.3, 0.15, 0.0),
                             (0.3, 0.15, 8.0)):
        hand = vps.handoff_pose(route, back, lat, math.radians(dyaw),
                                target=st.staging_pose)
        yaw_vs_stage = math.degrees(wrap(hand[2] - st.staging_pose[2]))
        anchor = (vps.PLAN_AT_STAGE
                  and vps.dist(hand, st.staging_pose) < 1.2
                  and abs(wrap(hand[2] - st.staging_pose[2]))
                  <= math.radians(vps.ANCHOR_YAW_TOL))
        print(f'\n--- back={back} lat={lat} dyaw={dyaw}deg ---')
        print(f'hand = ({hand[0]:+.2f}, {hand[1]:+.2f}, '
              f'{math.degrees(hand[2]):+.1f}deg)  '
              f'hand_yaw−stage_yaw = {yaw_vs_stage:+.1f}deg  锚定 = {anchor}')
        plan_start = tuple(st.staging_pose) if anchor else hand

        pts = [hand, st.goal_pose, st.staging_pose]
        cx = sum(p[0] for p in pts) / 3.0
        cy = sum(p[1] for p in pts) / 3.0
        half = max(15.0, max(vps.dist(p, (cx, cy)) for p in pts) + 6.0)
        grid = layout.occupancy_grid(
            vps.XY_RES / 2.0,
            roi=(cx - half, cx + half, cy - half, cy + half))
        planner = HybridAStar(grid, wheelbase=WHEELBASE,
                              max_steer=vps.PLAN_MAX_STEER,
                              xy_res=vps.XY_RES, th_res=vps.TH_RES,
                              clearance=vps.CLEARANCE, max_cusps=2)
        res, path, used = vps.plan_path(planner, plan_start, st)
        print(f'规划：{res.reason} len={res.length:.2f} m switch={res.n_switch} '
              f'两段式={used}')
        print(f'plan_start = ({plan_start[0]:+.2f}, {plan_start[1]:+.2f}, '
              f'{math.degrees(plan_start[2]):+.1f}deg)')
        print(f'path 前 8 点 / 首段切线:')
        for i, p in enumerate(path[:8]):
            ty = seg_yaw(path[i], path[i + 1]) if i + 1 < len(path) else float('nan')
            print(f'  [{i}] ({p[0]:+.2f}, {p[1]:+.2f}, {math.degrees(p[2]):+.1f}deg '
                  f'gear={p[3]:+d}) 切线={math.degrees(ty):+.1f}deg')
        first_tan = seg_yaw(path[0], path[1])
        phi0 = math.degrees(wrap(first_tan - hand[2]))
        print(f'起点切线 = {math.degrees(first_tan):+.1f}deg, '
              f'hand_yaw = {math.degrees(hand[2]):+.1f}deg, '
              f'φ(0) = 切线−hand_yaw = {phi0:+.1f}deg')

        # 第一拍的控制器内部量
        from parking_planner.controller import PathFollower
        f = PathFollower(wheelbase=WHEELBASE, max_steer=vps.PARK_MAX_STEER,
                         v_max=vps.PARK_V_MAX, steer_gain=vps.PARK_STEER_GAIN,
                         k_rate=vps.PARK_K_RATE, steer_rate=0.0,
                         preview=vps.PARK_PREVIEW,
                         goal_xy_tol=vps.GOAL_XY_TOL,
                         goal_yaw_tol=vps.GOAL_YAW_TOL)
        f.set_path(path)
        cmd = f.update(hand, 0.05, yaw_rate=None)
        d = f.dbg
        print(f'第一拍：e={d.get("e", 0):+.3f} φ={math.degrees(d.get("phi", 0)):+.1f}deg '
              f'κ={d.get("kappa", 0):+.3f} 参考yaw={math.degrees(d.get("thr", 0)):+.1f}deg '
              f'k_ref={d.get("k_ref", 0):+.3f} seg={d.get("seg")} t={d.get("t"):.2f}')


if __name__ == '__main__':
    main()
