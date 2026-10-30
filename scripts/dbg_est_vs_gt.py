#!/usr/bin/env python3
"""用 est(slam) 流投影到离线重建的 E2 锚定路径上，复算控制器的 s/e/φ，
与真机日志对齐（按 s 匹配），再与 gt 流同帧对比 —— 分离真实发散与测量假象。

est 在 slam map 帧 ≈ spawn 相对帧（全局刚体差 <0.03°/0.02 m，忽略）。
路径用相对帧（不加 SPAWN）。
"""
import bisect
import csv
import math
import os
import sys

WS = os.path.expanduser('~/apa_ws')
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
from parking_planner.geometry import wrap
from parking_planner.hybrid_astar import HybridAStar
from parking_planner.lot_layout import Layout, WHEELBASE
from parking_planner.route_planner import RouteGraph, default_graph_path

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds', 'city_parking_lot.sdf')
SPAWN = (-30.0, 15.65, 0.0)
POSE_CSV = '/tmp/pose_E2_a1.csv'

layout = Layout.from_world(WORLD).to_frame(SPAWN)
graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
st = layout.find_stall('E2')
route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
pts = [route[-1], st.goal_pose, st.staging_pose]
cx = sum(p[0] for p in pts) / 3.0
cy = sum(p[1] for p in pts) / 3.0
half = max(15.0, max(math.hypot(p[0] - cx, p[1] - cy) for p in pts) + 6.0)
grid = layout.occupancy_grid(0.125, roi=(cx - half, cx + half,
                                         cy - half, cy + half))
planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=0.42,
                      xy_res=0.25, th_res=math.radians(3.0),
                      clearance=0.15, max_cusps=2, clear_target=0.30)
res = planner.plan(tuple(st.staging_pose), st.goal_pose)
assert res.success
path = res.path            # 相对帧
gears = [p[3] for p in path]
sw = next((i for i in range(1, len(path)) if gears[i] != gears[0]), None)
seglen = [math.hypot(path[i + 1][0] - path[i][0],
                     path[i + 1][1] - path[i][1])
          for i in range(len(path) - 1)]
cum = [0.0]
for L in seglen:
    cum.append(cum[-1] + L)
print(f'路径 {res.length:.2f} m，cusp 在 i={sw} s={cum[sw]:.2f} m')


def project(x, y, seg_lo, seg_hi):
    best = None
    for i in range(seg_lo, min(seg_hi, len(path) - 1)):
        ax, ay = path[i][0], path[i][1]
        dx, dy = path[i + 1][0] - ax, path[i + 1][1] - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-12:
            continue
        t = ((x - ax) * dx + (y - ay) * dy) / seg2
        tc = max(0.0, min(1.0, t))
        d = math.hypot(x - (ax + dx * tc), y - (ay + dy * tc))
        if best is None or d < best[0]:
            best = (d, i, tc)
    if best is None:
        return None
    _d, i, t = best
    ax, ay = path[i][0], path[i][1]
    bx, by = path[i + 1][0], path[i + 1][1]
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
    px, py = ax + dx * t, ay + dy * t
    dth = wrap(path[i + 1][2] - path[i][2])
    thr = wrap(path[i][2] + dth * t)
    e = -math.sin(thr) * (x - px) + math.cos(thr) * (y - py)
    return cum[i] + math.sqrt(seg2) * t, e, thr


rows = list(csv.DictReader(open(POSE_CSV)))
gt = sorted([(float(r['t_stamp']), float(r['x']), float(r['y']), float(r['yaw']))
             for r in rows if r['src'] == 'gt'])
# est 已在 slam map 帧 ≈ spawn 相对帧（原点在出生点），直接用；
# gt 在 gz 世界帧，减 SPAWN 转到相对帧
est = sorted([(float(r['t_stamp']), float(r['x']), float(r['y']),
               float(r['yaw'])) for r in rows if r['src'] == 'est'])
gt_t = [p[0] for p in gt]

# est 流里找「接近 staging(相对帧) 1.2 m」的首帧，从那里开始扫
sx, sy = st.staging_pose[0], st.staging_pose[1]
t_start = next((t for t, x, y, yaw in est
                if math.hypot(x - sx, y - sy) < 1.2), None)
print(f'est 接近 staging 首帧 t={t_start:.2f}')

# 换挡：est 停车站（v<0.05 持续 0.6 s），且要求此前已沿 D 走过 >2 m
t_shift_est = None
prog = []
run_slow = 0.0
prev = None
s_max = 0.0
for k, (t, x, y, yaw) in enumerate(est):
    if t < t_start:
        prev = (t, x, y, yaw)
        continue
    pr = project(x, y, 0, sw + 1)
    if pr:
        s_max = max(s_max, pr[0])
    if prev is not None:
        dt_ = t - prev[0]
        if dt_ > 1e-3:
            v_ = math.hypot(x - prev[1], y - prev[2]) / dt_
            run_slow = run_slow + dt_ if v_ < 0.05 else 0.0
            if run_slow > 0.6 and s_max > 2.0:
                t_shift_est = t - run_slow / 2.0
                break
    prev = (t, x, y, yaw)
print(f'est 换挡停车站 t={t_shift_est:.2f}（此前 s_max={s_max:.2f}）')

print('\n  t_est    s_est   e_est   φ_est° |   s_gt    e_gt    φ_gt°')
nxt = t_start
for k, (t, x, y, yaw) in enumerate(est):
    if t < t_start or t > t_shift_est + 20.0:
        continue
    if t < nxt:
        continue
    nxt += 0.5
    seg_hi = sw + 1 if t <= t_shift_est else len(path)
    seg_lo = 0 if t <= t_shift_est else sw
    pr_e = project(x, y, seg_lo, seg_hi)
    j = bisect.bisect_left(gt_t, t)
    j = min(j, len(gt) - 1)
    pr_g = project(gt[j][1] - SPAWN[0], gt[j][2] - SPAWN[1], seg_lo, seg_hi)
    if pr_e and pr_g:
        print(f'{t:7.2f} {pr_e[0]:7.2f} {pr_e[1]:+7.3f} '
              f'{math.degrees(wrap(yaw - pr_e[2])):+7.2f} | '
              f'{pr_g[0]:7.2f} {pr_g[1]:+7.3f} '
              f'{math.degrees(wrap(gt[j][3] - pr_g[2])):+7.2f}')
