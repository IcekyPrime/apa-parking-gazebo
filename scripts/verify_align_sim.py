#!/usr/bin/env python3
"""离线验证「方案 A」：交接 → ALIGN 受控机动（对齐到预备位姿）→ 入位。

为什么要单独一个脚本：`verify_park_sim.py` 直接拿交接位姿去规划入位，测不到
ALIGN。方案 A 的核心假设是「先把车精确摆到预备位姿，入位就只剩一条直线」，
所以必须把整条链一起跑，而且要看**执行轨迹的真实车体间隙**。

ALIGN 的执行策略复刻 planner_node：分段滚动 —— 每段规划后只执行
`horizon` 米，然后从**自车实际位姿**重新规划，直到「距预备位姿 ≤ xy_tol 且
航向差 ≤ yaw_tol」。

用法：
    python3 scripts/verify_align_sim.py                 # 全部可用垂直车位
    python3 scripts/verify_align_sim.py W4
    HANDOFF_BACK=0.25 HANDOFF_LAT=0.25 python3 scripts/verify_align_sim.py
    ALIGN_HORIZON=9 python3 scripts/verify_align_sim.py   # 不分段，整条执行
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from parking_planner.geometry import body_gap, wrap                    # noqa: E402
from parking_planner.hybrid_astar import HybridAStar                   # noqa: E402
from parking_planner.lot_layout import (EFFECTIVE_MAX_STEER, Layout,    # noqa: E402
                                        WHEELBASE)
from parking_planner.route_planner import (RouteGraph,                 # noqa: E402
                                           default_graph_path)
import verify_park_sim as V                                            # noqa: E402

HORIZON = float(os.environ.get('ALIGN_HORIZON', '2.5'))
XY_TOL = float(os.environ.get('ALIGN_XY_TOL', '0.15'))
YAW_TOL = math.radians(float(os.environ.get('ALIGN_YAW_TOL', '5.0')))
MAX_SEG = int(os.environ.get('ALIGN_MAX_SEG', '10'))
TIME_LIMIT = float(os.environ.get('ALIGN_TIME', '45'))
PATIENCE = int(os.environ.get('ALIGN_PATIENCE', '2'))
# 每段结束时注入的跟踪残差（m / 度）：让离线更接近真机，0 = 关掉
PLANT_ERR = float(os.environ.get('ALIGN_PLANT_ERR', '0.12'))
PLANT_YAW = float(os.environ.get('ALIGN_PLANT_YAW', '1.5'))
# 入位规划转角上限，与 planner_node 的 plan_max_steer / verify_park_sim 一致
PLAN_MAX_STEER = float(os.environ.get('PLAN_MAX_STEER', '0.40'))


def trim(path, horizon):
    """截到 horizon 米弧长，但**不早于第一个换挡点**（与 planner_node._trim_path 一致）。

    真机 W5 的教训：多点机动是「先前进 2.5 m 拉开、再倒车 7.1 m」，而分段弧长
    也是 2.5 m 时，每段只走完「拉开」就重规划，重规划又是「拉开」—— 车在
    5.3~6.5 m 之间来回，永远到不了预备位姿。

    真机 W4 的第二个教训：那一步「拉开」只有 1.1 m（< MIN_SEG_LEN），只包第一把
    时每段仍然只走完 1.1 m 就被切掉，总长一直停 7.9~8.4 m 不收敛，6 段后超时。
    所以「太短的一把」要把下一把也包进来。
    """
    if len(path) < 3:
        return path
    MIN_SEG = 1.2
    arc, g0, seg_start, extended = 0.0, path[0][3], 0.0, False
    cap = horizon * 3.0
    for i in range(1, len(path)):
        arc += math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
        if path[i][3] == g0:
            continue
        leg_len = arc - seg_start
        g0, seg_start = path[i][3], arc
        if leg_len >= MIN_SEG:
            if extended or arc < horizon:
                horizon = arc + 0.5
            break
        horizon = min(arc + 0.5, cap)
        extended = True
        if arc >= cap:
            break
    out, acc = [path[0]], 0.0
    for p in path[1:]:
        acc += math.hypot(p[0] - out[-1][0], p[1] - out[-1][1])
        out.append(p)
        if acc >= horizon:
            break
    return out if len(out) >= 2 else path


def align_chain(planner, start, staging):
    """复刻 `_begin_align` + `_align_tick` + `_replan_align` 的滚动执行。

    终止条件（任一满足）：
      · 位姿到位：位置 ≤ XY_TOL 且航向差 ≤ YAW_TOL；
      · **不再改进**：连续 PATIENCE 段「距预备位姿」改善都 < 0.02 m —— 分段滚动
        在末段会停在「位置基本到了、航向差几度」的平台，再规划只是原地打转
        （实测有车位白白用掉 10 段、35 s）；
      · 段数/时间上限。
    """
    live = start
    traj = [live]
    segs, t_total = 0, 0.0
    prev_d = V.dist(live, staging)
    stall = 0
    while segs < MAX_SEG:
        d_now = V.dist(live, staging)
        if d_now <= XY_TOL and abs(wrap(live[2] - staging[2])) <= YAW_TOL:
            break
        if t_total > TIME_LIMIT or stall >= PATIENCE:
            break
        out = planner.plan(live, staging)
        if not out.success:
            break
        seg = trim(out.path, HORIZON)
        live, t, _ml, _sw, _cmd, tr = V.simulate(seg, live, tmax=30.0)
        traj += tr[1:]
        t_total += t
        segs += 1
        # 注入「每段结束时的跟踪残差」。离线模型跟得太准，会掩盖真机上
        # 「拉开→倒车」被分段截断后原地打转这一类问题（真机 W5 就是这么失败的）。
        if PLANT_ERR > 0.0:
            sgn = 1.0 if segs % 2 else -1.0
            live = (live[0] - math.sin(live[2]) * PLANT_ERR * sgn,
                    live[1] + math.cos(live[2]) * PLANT_ERR * sgn,
                    wrap(live[2] + math.radians(PLANT_YAW) * sgn))
            traj.append(live)
        d_new = V.dist(live, staging)
        stall = stall + 1 if (prev_d - d_new) < 0.02 else 0
        prev_d = d_new
    ok = (V.dist(live, staging) <= XY_TOL
          and abs(wrap(live[2] - staging[2])) <= YAW_TOL)
    return live, segs, t_total, traj, ok


def main():
    only = sys.argv[1] if len(sys.argv) > 1 else None
    layout = Layout.from_world(V.WORLD).to_frame(V.SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(V.SPAWN)

    ids = []
    kinds = os.environ.get('ALIGN_KINDS', 'perpendicular')
    for st in layout.stalls:
        if kinds != 'all' and st.kind != kinds:
            continue
        if body_gap(st.goal_pose, layout.obstacles) <= V.GOAL_MIN_GAP:
            continue
        if only and st.id != only:
            continue
        ids.append(st.id)

    print(f'ALIGN 参数：每段 {HORIZON} m，到点判据 {XY_TOL} m / '
          f'{math.degrees(YAW_TOL):.0f}°，最多 {MAX_SEG} 段；'
          f'交接位姿回退 {V.HANDOFF_BACK} m / 横偏 {V.HANDOFF_LAT} m')
    print(f'{"车位":5s} {"段":3s} {"耗时":7s} {"距预备":8s} {"航向差":7s} '
          f'{"入位换挡":8s} {"终点误差":9s} {"航向":7s} {"最小间隙":10s} 判定')
    ok = total = scraped = 0
    for tid in ids:
        st = layout.find_stall(tid)
        # 起点用 map 帧原点 = 真实出生点（layout/graph 都已 `.to_frame(V.SPAWN)`）
        route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
        if not route:
            print(f'{tid:5s} 路线规划失败')
            continue
        hand = V.handoff_pose(route, V.HANDOFF_BACK, V.HANDOFF_LAT)
        pts = [hand, st.goal_pose, st.staging_pose]
        cx = sum(p[0] for p in pts) / 3.0
        cy = sum(p[1] for p in pts) / 3.0
        half = max(15.0, max(math.hypot(p[0] - cx, p[1] - cy) for p in pts) + 6.0)
        grid = layout.occupancy_grid(V.XY_RES / 2.0,
                                     roi=(cx - half, cx + half, cy - half, cy + half))
        pl = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=PLAN_MAX_STEER,
                         xy_res=V.XY_RES, th_res=V.TH_RES,
                         clearance=V.CLEARANCE, max_cusps=3)

        live, segs, t_align, traj_a, aligned = align_chain(pl, hand, st.staging_pose)
        d_stage = V.dist(live, st.staging_pose)
        d_yaw = math.degrees(abs(wrap(live[2] - st.staging_pose[2])))
        # ALIGN 之后：从**实际位姿**规划入位（和 _plan 一致，余量优先）
        res, path, used = V.plan_path(pl, live, st)
        if not res.success:
            print(f'{tid:5s} {segs:3d} {t_align:6.1f}s {d_stage:7.2f} {d_yaw:6.1f}°  '
                  f'  入位规划失败')
            continue
        final, _t, _ml, shifts, _cmd, traj_b = V.simulate(path, live)
        traj = traj_a + traj_b[1:]
        e = V.dist(final, st.goal_pose)
        ey = math.degrees(abs(wrap(final[2] - st.goal_pose[2])))
        gap, _at = V.traj_gap(traj, layout.obstacles)
        good = e <= 0.35 and ey <= 9.0 and gap > 0.0
        total += 1
        ok += 1 if good else 0
        if gap <= 0.0:
            scraped += 1
        if gap <= 0.0:
            note = '剐蹭!'
        elif not good:
            note = '超差'
        elif not aligned:
            note = f'对齐偏差大({d_stage:.2f} m/{d_yaw:.0f}°)'
        else:
            note = ''
        print(f'{tid:5s} {segs:3d} {t_align:6.1f}s {d_stage:7.2f} {d_yaw:6.1f}° '
              f'{shifts:8d} {e:8.2f} {ey:6.1f}° {gap:+10.2f} {note}')
    if total:
        print(f'\n达标 {ok}/{total}（判据：终点 ≤0.35 m / ≤9° + 全程车体间隙 >0；'
              f'剐蹭 {scraped} 个）')


if __name__ == '__main__':
    main()
