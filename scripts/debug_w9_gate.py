#!/usr/bin/env python3
"""W9 首换挡门限调试：逐步执行 follower，打印每次换挡判定内部量。"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

import verify_park_sim as vps                                    # noqa: E402
from parking_planner.controller import PathFollower              # noqa: E402
from parking_planner.hybrid_astar import HybridAStar            # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE        # noqa: E402
from parking_planner.geometry import wrap                        # noqa: E402
from parking_planner.route_planner import RouteGraph, default_graph_path  # noqa: E402

BACK, LAT, DYAW = 0.3, 0.15, 0.0

layout = Layout.from_world(vps.WORLD).to_frame(vps.SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(vps.SPAWN)
st = layout.find_stall('W9')
route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
hand = vps.handoff_pose(route, BACK, LAT, math.radians(DYAW),
                        target=st.staging_pose)
pts = [hand, st.goal_pose, st.staging_pose]
cx = sum(p[0] for p in pts) / 3.0
cy = sum(p[1] for p in pts) / 3.0
half = max(15.0, max(vps.dist(p, (cx, cy)) for p in pts) + 6.0)
grid = layout.occupancy_grid(vps.XY_RES / 2.0,
                             roi=(cx - half, cx + half, cy - half, cy + half))
planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=vps.PLAN_MAX_STEER,
                       xy_res=vps.XY_RES, th_res=vps.TH_RES,
                       clearance=vps.CLEARANCE, max_cusps=2)
res, path, used = vps.plan_path(planner, hand, st)
print(f'plan: {res.reason} len={res.length:.2f} switch={res.n_switch} '
      f'used_stage={used}')
print(f'hand = ({hand[0]:.3f}, {hand[1]:.3f}, {math.degrees(hand[2]):.1f}deg)')
for i, p in enumerate(path[:6]):
    print(f'  path[{i}] = ({p[0]:.3f}, {p[1]:.3f}, {math.degrees(p[2]):.1f}deg '
          f'gear={p[3]:+d} k={p[4]:+.4f})')

f = PathFollower(wheelbase=WHEELBASE, max_steer=vps.PARK_MAX_STEER,
                 v_max=vps.PARK_V_MAX, steer_gain=vps.PARK_STEER_GAIN,
                 k_rate=vps.PARK_K_RATE, steer_rate=0.0,
                 preview=vps.PARK_PREVIEW,
                 goal_xy_tol=vps.GOAL_XY_TOL,
                 goal_yaw_tol=vps.GOAL_YAW_TOL)
f.set_path(path)
print(f'_first_cusp_idx={f._first_cusp_idx} _first_leg={f._first_leg:.3f} '
      f'shift_gate_arc={f.shift_gate_arc}')

x, y, th = hand
dt = 0.05
for step in range(400):
    cmd = f.update((x, y, th), dt, yaw_rate=None)
    if cmd.done or cmd.stuck:
        print(f'step {step}: done={cmd.done} stuck={cmd.stuck}')
        break
    # 手动复刻被控对象（与 simulate 相同）
    v, w = cmd.v, cmd.omega
    d = math.atan(w * WHEELBASE / v) if abs(v) > 1e-3 else 0.0
    om = v * math.tan(d) / WHEELBASE * vps.UNDER
    th = wrap(th + om * dt)
    x += v * math.cos(th) * dt
    y += v * math.sin(th) * dt
    if step < 16 or step % 20 == 0:
        print(f'step {step:3d} gear={cmd.gear:+d} v={cmd.v:+.2f} '
              f'e={cmd.lateral_error:+.3f} rem={cmd.remaining:6.2f} '
              f'idx={f.idx} proj_seg={f._proj_seg} t={f._proj_t:.2f} '
              f'gate_first={(f._first_cusp_idx, round(f._first_leg, 3))}')
