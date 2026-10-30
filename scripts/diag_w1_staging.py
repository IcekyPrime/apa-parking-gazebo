#!/usr/bin/env python3
"""W1 在线预备位姿搜索失败的离线复现与根因对比。

真机 W1 日志：「在线搜索没找到合法预备位姿，沿用离线表的值」→ 回退到
同轴默认位姿（朝东、与过道垂直）→ 路线无法延伸 → 交接点距预备位姿
3.5 m / 89° → 直连 2 换挡满曲率解 → 跟踪 e=0.59 m → 间隙 0.03 m。

怀疑根因：search_staging 的候选在 free/plan 校验时沿**车位轴**做了
中心→后轴换算（§6.29 修改引入），而正确的换算应沿**候选自身朝向**
（过道方向，与车位轴垂直）—— 校验位姿与真实位姿偏差约 1.9 m。
本脚本对同一批候选并排跑两套检查，验证该假设。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.hybrid_astar import HybridAStar   # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE  # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
# 真机 batch_real 的出生点（SX=-30），保证与真机日志同一 map 帧
SPAWN = (-30.0, 15.65, 0.0)
XY_RES, TH_RES = 0.25, math.radians(3.0)
WRAP = lambda a: (a + math.pi) % (2 * math.pi) - math.pi   # noqa: E731

TID = sys.argv[1] if len(sys.argv) > 1 else 'W1'
ALONG = 9.0
R_LIST = (5.74, 6.39, 7.00)
APPROACH_YAW = -math.pi / 2
YAW_TOL = math.radians(15.0)


def main():
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    st = layout.find_stall(TID)
    gx, gy, gyaw = st.goal_pose
    ux, uy = math.cos(gyaw), math.sin(gyaw)
    nx, ny = -uy, ux
    ax = WHEELBASE * 0.5
    gx_r, gy_r = gx - ux * ax, gy - uy * ax          # 后轴语义 goal（正确）

    half = 25.0
    roi = (gx - half, gx + half, gy - half, gy + half)
    grid = layout.occupancy_grid(XY_RES / 2.0, roi=roi)
    planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=0.42,
                          xy_res=XY_RES, th_res=TH_RES, clearance=0.15,
                          max_cusps=2, clear_target=0.30)
    saved = planner.max_iter
    planner.max_iter = 1

    print(f'=== {TID} goal=({gx:.2f},{gy:.2f},{math.degrees(gyaw):.0f}deg) '
          f'过道行进 {-90}deg ===')
    hdr = (f'{"along":>6s} {"R":>5s} {"side":>4s} {"h":>3s} | '
           f'{"bug_free":>8s} {"bug_plan":>8s} {"bug_sw":>6s} {"bug_clr":>7s} | '
           f'{"fix_free":>8s} {"fix_plan":>8s} {"fix_sw":>6s} {"fix_clr":>7s} '
           f'{"fix_body":>8s}')
    print(hdr)
    n_bug_ok = n_fix_ok = 0
    try:
        for sl in (0.0, 0.6, -0.6):
            for R in R_LIST:
                for side in (1.0, -1.0):
                    # 现行（buggy）：候选 = 后轴goal + 偏移（沿车位轴影子系）
                    bx = gx_r + (ALONG + sl) * ux + side * R * nx
                    by = gy_r + (ALONG + sl) * uy + side * R * ny
                    # 修正（fixed）：中心候选 = 中心goal + 偏移
                    cx_ = gx + (ALONG + sl) * ux + side * R * nx
                    cy_ = gy + (ALONG + sl) * uy + side * R * ny
                    for h in (1.0, -1.0):
                        syaw = math.atan2(h * ny, h * nx)
                        if abs(WRAP(syaw - APPROACH_YAW)) > YAW_TOL:
                            continue
                        # 修正版后轴：沿候选自身朝向后移
                        fx = cx_ - math.cos(syaw) * ax
                        fy = cy_ - math.sin(syaw) * ax
                        row = [f'{ALONG + sl:6.1f} {R:5.2f} {side:+4.0f} '
                               f'{h:+3.0f} |']
                        # --- buggy ---
                        bf = planner.free(bx, by, syaw)
                        bp = planner.plan((bx, by, syaw), (gx_r, gy_r, gyaw))
                        bs = bp.n_switch if bp.success else -1
                        bc = (planner.min_clearance(list(bp.path))
                              if bp.success else float('nan'))
                        if bf and bp.success and bs == 0 and bc >= 0.30:
                            n_bug_ok += 1
                        row.append(f'{str(bf):>8s} {str(bp.success):>8s} '
                                   f'{bs:6d} {bc:+7.2f} |')
                        # --- fixed ---
                        ff = planner.free(fx, fy, syaw)
                        fp = planner.plan((fx, fy, syaw), (gx_r, gy_r, gyaw))
                        fs = fp.n_switch if fp.success else -1
                        fc = (planner.min_clearance(list(fp.path))
                              if fp.success else float('nan'))
                        fb = 0.0
                        if fp.success:
                            from parking_planner.geometry import body_gap
                            fb = min(body_gap((p[0], p[1], p[2]),
                                              layout.obstacles)
                                     for p in fp.path)
                        if ff and fp.success and fs == 0 and fc >= 0.30:
                            n_fix_ok += 1
                            row.append(f'{str(ff):>8s} {str(fp.success):>8s} '
                                       f'{fs:6d} {fc:+7.2f} {fb:+8.2f} <<')
                        else:
                            row.append(f'{str(ff):>8s} {str(fp.success):>8s} '
                                       f'{fs:6d} {fc:+7.2f} {fb:+8.2f}')
                        print(''.join(row))
    finally:
        planner.max_iter = saved
    print(f'\n现行换算可行候选：{n_bug_ok}    修正换算可行候选：{n_fix_ok}')


if __name__ == '__main__':
    main()
