"""E2 间隙损失定位：规划路径几何间隙 vs 执行轨迹间隙。

回答两个问题：
1. 规划路径本身离邻车多近？（几何问题 → 规划侧修）
2. 执行比规划弯多少 / 偏多少？（跟踪问题 → 控制侧修）
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
from parking_planner.geometry import body_gap

import verify_park_sim as V

STALL = sys.argv[1] if len(sys.argv) > 1 else 'E2'

layout = Layout.from_world(V.WORLD).to_frame(V.SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(V.SPAWN)
st = layout.find_stall(STALL)
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
                      clearance=V.CLEARANCE, max_cusps=2, clear_target=0.30)
res, path, used = V.plan_path(planner, hand, st)
print(f'plan: success={res.success} reason={res.reason} n_switch={res.n_switch} '
      f'near_stage={V.dist(hand, st.staging_pose) < 0.8} '
      f'hand=({hand[0]:.3f},{hand[1]:.3f},{math.degrees(hand[2]):.1f}deg) '
      f'staging=({st.staging_pose[0]:.3f},{st.staging_pose[1]:.3f})')
if not path:
    sys.exit('EMPTY PATH')

# 1) 规划路径逐点间隙
pg_best, pg_i = 1e9, -1
for i, p in enumerate(path):
    g = body_gap((p[0], p[1], p[2]), layout.obstacles)
    if g < pg_best:
        pg_best, pg_i = g, i
sp = 0.0
for i in range(1, pg_i + 1):
    sp += math.hypot(path[i][0] - path[i-1][0], path[i][1] - path[i-1][1])
print(f'[{STALL}] 规划路径: {len(path)} 点, 总长 {sum(math.hypot(path[i][0]-path[i-1][0], path[i][1]-path[i-1][1]) for i in range(1,len(path))):.2f} m')
print(f'  规划最小间隙 {pg_best:+.3f} m @ s={sp:.2f} m '
      f'(κ={path[pg_i][4]:+.4f}, gear={path[pg_i][3]})')

# 2) 无噪声执行
final, t, maxlat, maxlat_true, shifts, cmd, traj = V.simulate(path, hand)
eg_best, eg_i = 1e9, -1
for i, p in enumerate(traj):
    g = body_gap(p, layout.obstacles)
    if g < eg_best:
        eg_best, eg_i = g, i
print(f'  执行最小间隙 {eg_best:+.3f} m @ traj[{eg_i}/{len(traj)}] '
      f'({100.0*eg_i/len(traj):.0f}%) pose=({traj[eg_i][0]:.3f},'
      f'{traj[eg_i][1]:.3f},{math.degrees(traj[eg_i][2]):.1f}deg)')
print(f'  → 间隙损失（规划-执行）= {pg_best - eg_best:+.3f} m')

# 3) 间隙敏感区（gap<0.30 的执行位姿）与最近规划点对比
print('\n执行位姿 vs 最近规划点（间隙<0.35 m 区段，每 8 帧取 1）:')
print(f'{"i":>5s} {"ex":>7s} {"ey":>7s} {"eθ°":>6s} {"gap":>6s} '
      f'{"near_s":>7s} {"Δn":>6s} {"Δl":>6s}')
for i in range(max(0, eg_i - 60), min(len(traj), eg_i + 60)):
    p = traj[i]
    g = body_gap(p, layout.obstacles)
    if g > 0.35:
        continue
    # 找最近规划点及其切向/法向偏差
    best_j, best_d = -1, 1e9
    for j, q in enumerate(path):
        d = math.hypot(q[0] - p[0], q[1] - p[1])
        if d < best_d:
            best_d, best_j = d, j
    q = path[best_j]
    thr = q[2]
    dn = -(p[0] - q[0]) * math.sin(thr) + (p[1] - q[1]) * math.cos(thr)
    dl = (p[0] - q[0]) * math.cos(thr) + (p[1] - q[1]) * math.sin(thr)
    if i % 8:
        continue
    print(f'{i:5d} {p[0]:7.3f} {p[1]:7.3f} {math.degrees(p[2]):6.1f} {g:+6.3f} '
          f'{best_j:7d} {dn:+6.3f} {dl:+6.3f}')
