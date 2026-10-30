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

layout = Layout.from_world(V.WORLD).to_frame(V.SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(V.SPAWN)
st = layout.find_stall('E2')
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
res, path, used = V.plan_path(planner, hand, st)
final, t, maxlat, maxlat_true, shifts, cmd, traj = V.simulate(path, hand)
g = st.goal_pose
print(f'final=({final[0]:.3f},{final[1]:.3f},{math.degrees(final[2]):.1f}deg) '
      f'goal=({g[0]:.3f},{g[1]:.3f}) err={math.hypot(final[0]-g[0],final[1]-g[1]):.3f}')
print(f'goal-pose body gap: {body_gap(g, layout.obstacles):+.3f}')
print(f'final-pose body gap: {body_gap(final, layout.obstacles):+.3f}')
print('gap along trajectory (every ~0.5 m):')
best, best_i = 1e9, -1
for i, (x, y, th) in enumerate(traj):
    gp = body_gap((x, y, th), layout.obstacles)
    if gp < best:
        best, best_i = gp, i
    if i % 10 == 0:
        print(f'  i={i:4d} ({x:7.3f},{y:8.3f},{math.degrees(th):7.1f}deg) gap={gp:+.3f}')
print(f'MIN gap {best:+.3f} at traj[{best_i}] of {len(traj)} '
      f'({100.0*best_i/len(traj):.0f}% through)')
x, y, th = traj[best_i]
print(f'  min-gap pose: ({x:.3f},{y:.3f},{math.degrees(th):.1f}deg)')
