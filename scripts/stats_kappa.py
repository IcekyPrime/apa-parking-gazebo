#!/usr/bin/env python3
"""全部垂直车位直连解的曲率统计：谁在「满曲率 1 换挡」上运行（E2 同款隐患）。

E2 真机发散的根源是规划曲率 0.1747 占执行限幅 0.2019 的 86%，反馈余量
只剩 0.019 —— 换挡瞬态 0.16 m 偏差就让指令顶死、偏差单调积累（§6.25）。
任何车位的解若同样贴满曲率，真机通过就只是「偏差方向恰好落在收敛侧」。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import body_gap, wrap     # noqa: E402
from parking_planner.hybrid_astar import HybridAStar    # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE  # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)
XY_RES, TH_RES = 0.25, math.radians(3.0)
CLEARANCE = 0.15
STEER = float(os.environ.get('PLAN_MAX_STEER', '0.44'))
KMAX = 0.2019          # 执行限幅（FOLLOW_MAX_STEER_PARK=0.50）


def path_kappa(path, q=0.95):
    ks = []
    for a, b in zip(path, path[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if d < 1e-6 or a[3] != b[3]:
            continue
        ks.append(abs(wrap(b[2] - a[2])) / d)
    return sorted(ks)[min(len(ks) - 1, int(q * len(ks)))] if ks else 0.0


def main():
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    print(f'plan_max_steer={STEER}（κ_plan={math.tan(STEER) / WHEELBASE:.4f}，'
          f'执行限幅 {KMAX}，反馈余量上限 '
          f'{KMAX / 1.04 - math.tan(STEER) / WHEELBASE:.4f}）\n')
    print(f'{"车位":5s} {"换挡":4s} {"长度":7s} {"余量":7s} {"κ95":7s} '
          f'{"κ/限幅":7s} {"反馈余量":8s} 隐患')
    for st in layout.stalls:
        if st.kind != 'perpendicular':
            continue
        if body_gap(st.goal_pose, layout.obstacles) <= 0.02:
            continue
        gx, gy, gyaw = st.goal_pose
        sx, sy, syaw = st.staging_pose
        grid = layout.occupancy_grid(
            XY_RES / 2.0,
            roi=(min(sx, gx) - 20, max(sx, gx) + 20,
                min(sy, gy) - 20, max(sy, gy) + 20))
        planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=STEER,
                              xy_res=XY_RES, th_res=TH_RES,
                              clearance=CLEARANCE, max_cusps=2)
        res = planner.plan((sx, sy, syaw), (gx, gy, gyaw))
        if not res.success:
            print(f'{st.id:5s} 规划失败: {res.reason}')
            continue
        c = planner.min_clearance(list(res.path))
        k = path_kappa(list(res.path))
        margin = KMAX / 1.04 - k
        risk = '⚠ 满曲率' if margin < 0.025 else ''
        print(f'{st.id:5s} {res.n_switch:4d} {res.length:7.1f} {c:+7.2f} '
              f'{k:7.4f} {k / KMAX:6.0%} {margin:8.4f} {risk}')


if __name__ == '__main__':
    main()
