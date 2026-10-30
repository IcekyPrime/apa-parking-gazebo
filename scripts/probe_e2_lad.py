#!/usr/bin/env python3
"""插桩 PathFollower._lookahead：逐帧记录 seg_i/seg_t/rest_total/gear_cut/返回值，
定位「预瞄在 remaining≈preview 处死亡（la_d=0）」的确切原因。"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

# 噪声环境变量必须在 import verify_park_sim 之前设好（模块级读取）
os.environ.setdefault('POSE_DRIFT_AMP', '0.15')
os.environ.setdefault('POSE_NOISE', '0.03')
os.environ.setdefault('POSE_YAW_NOISE', '0.2')
os.environ.setdefault('POSE_SEED', sys.argv[2] if len(sys.argv) > 2 else '5')

from parking_planner.controller import PathFollower           # noqa: E402
from parking_planner.hybrid_astar import HybridAStar           # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE       # noqa: E402
from parking_planner.route_planner import (RouteGraph,          # noqa: E402
                                           default_graph_path)
from verify_park_sim import (CLEARANCE, PLAN_MAX_STEER, TH_RES, XY_RES,  # noqa: E402
                             WORLD, SPAWN, handoff_pose, plan_path,
                             simulate)

frames = []      # 每次 update：((x,y,th), la 记录数)


def make_patch(log):
    orig = PathFollower._lookahead

    def patched(self, seg_i, seg_t, dist, anchor_end=True):
        n = len(self.path)
        i = max(0, min(seg_i, n - 2))
        left = (1.0 - seg_t) * self._seg_len(i)
        rest = left
        j = i + 1
        cut = False
        while j < n - 1:
            if self.path[j + 1][3] != self.gear:
                cut = True
                break
            rest += self._seg_len(j)
            j += 1
        res = orig(self, seg_i, seg_t, dist, anchor_end)
        log.append(dict(seg_i=seg_i, seg_t=round(seg_t, 3), gear=self.gear,
                        dist=round(dist, 3), rest=round(rest, 3), cut=cut,
                        res=None if res is None else
                        (round(res[0], 2), round(res[1], 2))))
        return res
    return orig, patched


orig_upd = PathFollower.update
la_log = []


def upd(self, pose, dt, speed_scale=1.0, yaw_rate=None):
    n0 = len(la_log)
    r = orig_upd(self, pose, dt, speed_scale, yaw_rate)
    frames.append((pose, len(la_log) - n0, r.remaining, r.gear))
    return r


def main():
    PathFollower.update = upd
    orig_la, patched = make_patch(la_log)
    PathFollower._lookahead = patched

    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
    tid = sys.argv[1] if len(sys.argv) > 1 else 'E2'
    st = layout.find_stall(tid)
    route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
    hand = handoff_pose(route, 0.0, 0.0, 0.0, target=st.staging_pose)
    pts = [hand, st.goal_pose, st.staging_pose]
    cx = sum(p[0] for p in pts) / 3.0
    cy = sum(p[1] for p in pts) / 3.0
    grid = layout.occupancy_grid(
        XY_RES / 2.0, roi=(cx - 25.0, cx + 25.0, cy - 25.0, cy + 25.0))
    planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=PLAN_MAX_STEER,
                          xy_res=XY_RES, th_res=TH_RES,
                          clearance=CLEARANCE if st.kind == 'perpendicular' else 0.0,
                          max_cusps=2 if st.kind == 'perpendicular' else 3)
    res, path, used = plan_path(planner, hand, st)
    print(f'{tid}: {res.reason} len={res.length:.2f} n_sw={res.n_switch} '
          f'pts={len(path)}')
    final, t_end, maxlat, maxlat_true, shifts, cmd, traj = simulate(path, hand)
    e = math.hypot(final[0] - st.goal_pose[0], final[1] - st.goal_pose[1])
    ey = math.degrees(abs(math.atan2(math.sin(final[2] - st.goal_pose[2]),
                                     math.cos(final[2] - st.goal_pose[2]))))
    print(f'final err {e:.2f} m / {ey:.1f}°  stuck={cmd.stuck} '
          f'frames={len(frames)} la_calls={len(la_log)}')

    # 只看 remaining < 3.2 的帧
    PathFollower._lookahead = orig_la
    PathFollower.update = orig_upd
    print(f'{"t":>6s} {"x":>7s} {"y":>7s} {"seg_i":>5s} {"seg_t":>5s} '
          f'{"gear":>4s} {"rest":>6s} {"cut":>4s} {"la":>14s}')
    k = 0
    for fi, (pose, nla, rem, gear) in enumerate(frames):
        if rem > 3.2 or rem < 0.02:
            k += nla
            continue
        rec = la_log[k] if nla else None
        t = fi * 0.05
        if fi % 2 == 0 or rec is None or rec['res'] is None:
            la_s = ('-' if rec is None else
                    f"({rec['res'][0]:.1f},{rec['res'][1]:.1f})" if rec['res']
                    else 'None')
            print(f'{t:6.2f} {pose[0]:7.2f} {pose[1]:7.2f} '
                  f'{rec["seg_i"] if rec else -1:5d} '
                  f'{rec["seg_t"] if rec else 0:5.2f} {gear:4d} '
                  f'{rec["rest"] if rec else 0:6.2f} '
                  f'{str(rec["cut"]) if rec else "-":>4s} {la_s:>14s}')
        k += nla


if __name__ == '__main__':
    main()
