#!/usr/bin/env python3
"""E2 真机 a1 的位姿流 vs 离线重建路径：分离「真实偏差 / 测量假象 / 执行欠转」。

做法：
  1. 用与真机一致的配置离线重建 E2 锚定解（staging→goal 直连，max_steer 0.42），
     得到 15.4 m 路径（map 帧，spawn=(-30, 15.65, 0)）。
  2. 读 pose_audit 的 /tmp/pose_E2_a1.csv（gt=/odom_gt 真值, est=/vehicle_pose）。
  3. 每帧投影到路径（最近段垂足），输出 s(进度)、e_true(gt)、e_est(est)、yaw。
  4. gt yaw 差分 → 真实偏航率 ω_true，与日志的指令 ω 对照得执行比。
"""
import csv
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import wrap                      # noqa: E402
from parking_planner.hybrid_astar import HybridAStar          # noqa: E402
from parking_planner.lot_layout import Layout, WHEELBASE      # noqa: E402
from parking_planner.route_planner import RouteGraph, default_graph_path  # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds', 'city_parking_lot.sdf')
SPAWN = (-30.0, 15.65, 0.0)          # batch_real 的 SX/SY
POSE_CSV = os.environ.get('POSE_CSV', '/tmp/pose_E2_a1.csv')
OUT_CSV = os.environ.get('OUT_CSV', '/tmp/e2_pose_analysis.csv')


def wrap_d(a):
    return a * 57.29577951308232


def project(path, x, y):
    """全路径最近段投影：返回 (s, e, seg_yaw)。"""
    best = None
    acc = 0.0
    for i in range(len(path) - 1):
        ax, ay = path[i][0], path[i][1]
        dx, dy = path[i + 1][0] - ax, path[i + 1][1] - ay
        seg2 = dx * dx + dy * dy
        if seg2 < 1e-12:
            acc += math.sqrt(seg2)
            continue
        t = ((x - ax) * dx + (y - ay) * dy) / seg2
        tc = max(0.0, min(1.0, t))
        px, py = ax + dx * tc, ay + dy * tc
        d = math.hypot(x - px, y - py)
        if best is None or d < best[0]:
            dth = wrap(path[i + 1][2] - path[i][2])
            best = (d, acc + math.sqrt(seg2) * tc, tc,
                    wrap(path[i][2] + dth * tc), t)
        acc += math.sqrt(seg2)
    if best is None:
        return 0.0, 0.0, path[0][2]
    d, s, tc, thr, _ = best
    e = -math.sin(thr) * (x - (path[0][0])) * 0 + math.cos(thr) * 0  # placeholder
    return s, d, thr


def signed_e(path, seg_i, tc, x, y):
    ax, ay = path[seg_i][0], path[seg_i][1]
    bx, by = path[seg_i + 1][0], path[seg_i + 1][1]
    dx, dy = bx - ax, by - ay
    seg2 = dx * dx + dy * dy
    t = max(0.0, min(1.0, ((x - ax) * dx + (y - ay) * dy) / seg2))
    px, py = ax + dx * t, ay + dy * t
    dth = wrap(path[seg_i + 1][2] - path[seg_i][2])
    thr = wrap(path[seg_i][2] + dth * t)
    e = -math.sin(thr) * (x - px) + math.cos(thr) * (y - py)
    s_off = math.sqrt(seg2) * t
    return e, thr, s_off


def main():
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
    st = layout.find_stall('E2')
    route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
    hand = route[-1]
    pts = [hand, st.goal_pose, st.staging_pose]
    cx = sum(p[0] for p in pts) / 3.0
    cy = sum(p[1] for p in pts) / 3.0
    half = max(15.0, max(math.hypot(p[0] - cx, p[1] - cy) for p in pts) + 6.0)
    grid = layout.occupancy_grid(0.125, roi=(cx - half, cx + half,
                                             cy - half, cy + half))
    planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=0.42,
                          xy_res=0.25, th_res=math.radians(3.0),
                          clearance=0.15, max_cusps=2, clear_target=0.30)
    res = planner.plan(tuple(st.staging_pose), st.goal_pose)
    if not res.success:
        print('锚定解规划失败:', res.reason)
        return
    # 相对帧（spawn 为原点）→ gz 世界帧：pose_audit 的 gt/est 都在世界帧
    path = [(p[0] + SPAWN[0], p[1] + SPAWN[1], p[2], p[3],
             p[4] if len(p) >= 5 else 0.0) for p in res.path]
    print(f'锚定解：{res.length:.2f} m，换挡 {res.n_switch}，'
          f'{len(path)} 点，reason={res.reason}')
    seglen = [math.hypot(path[i + 1][0] - path[i][0],
                         path[i + 1][1] - path[i][1])
              for i in range(len(path) - 1)]
    total = sum(seglen)
    gears = [p[3] for p in path]
    sw = next((i for i in range(1, len(path)) if gears[i] != gears[0]), None)
    print(f'  首段(D) {sum(seglen[:sw]):.2f} m，之后 R 段 {total - sum(seglen[:sw]):.2f} m')

    rows = list(csv.DictReader(open(POSE_CSV)))
    gt = [(float(r['t_stamp']), float(r['x']), float(r['y']), float(r['yaw']))
          for r in rows if r['src'] == 'gt']
    est = [(float(r['t_stamp']), float(r['x']), float(r['y']), float(r['yaw']))
           for r in rows if r['src'] == 'est']
    gt.sort(key=lambda p: p[0])
    est.sort(key=lambda p: p[0])
    print(f'pose 流：gt {len(gt)} 帧（{gt[0][0]:.1f}~{gt[-1][0]:.1f} s），'
          f'est {len(est)} 帧')

    def proj(pose, seg_lo, seg_hi):
        """段范围 [seg_lo, seg_hi) 内最近段投影：复刻控制器的档位过滤。"""
        x, y = pose[1], pose[2]
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
            return 0.0, 0.0, path[seg_lo][2], 1e9
        d, i, t = best
        e, thr, s_off = signed_e(path, i, t, x, y)
        s = sum(seglen[:i]) + s_off
        return s, e, thr, d

    # 入位段起点：gt 流中车首次进入路径起点（staging）1.2 m 内
    t0 = None
    i0 = None
    for k, p in enumerate(gt):
        if math.hypot(p[1] - path[0][0], p[2] - path[0][1]) < 1.2:
            t0, i0 = p[0], k
            break
    if t0 is None:
        print('gt 流里没找到接近 staging 的帧！')
        return
    # 跟踪起点：t0 之后车首次动起来（速度 >0.1 m/s）—— 交接后有 ~6 s 停车
    t1 = t0
    for k in range(i0, len(gt) - 1):
        dt_ = gt[k + 1][0] - gt[k][0]
        if dt_ > 1e-3:
            v_ = math.hypot(gt[k + 1][1] - gt[k][1],
                            gt[k + 1][2] - gt[k][2]) / dt_
            if v_ > 0.1:
                t1 = gt[k][0]
                break
    print(f'入位段起点（gt 距 staging<1.2 m 首帧）：t={t0:.2f}，'
          f'起步（v>0.1）：t={t1:.2f}')
    # 换挡分界：起步后首个「速度≈0 且持续 >0.6 s」的停车站（shift_hold）
    t_shift = t1 + 30.0
    run_slow = 0.0
    for k in range(i0, len(gt) - 1):
        if gt[k][0] < t1:
            continue
        dt_ = gt[k + 1][0] - gt[k][0]
        if dt_ > 1e-3:
            v_ = math.hypot(gt[k + 1][1] - gt[k][1],
                            gt[k + 1][2] - gt[k][2]) / dt_
            run_slow = run_slow + dt_ if v_ < 0.05 else 0.0
            if run_slow > 0.6 and gt[k][0] > t1 + 2.0:
                t_shift = gt[k][0] - run_slow / 2.0
                break
    print(f'换挡停车站：t={t_shift:.2f}（D 段投影用 [t1,t_shift]，R 段用之后）')
    projD = lambda p: proj(p, 0, sw + 1)          # D 段 + 跨换挡段
    projR = lambda p: proj(p, sw, len(path))      # R 段

    out = open(OUT_CSV, 'w', newline='')
    w = csv.writer(out)
    w.writerow(['t_rel', 's_gt', 'e_true', 'yaw_gt', 'w_true', 'thr_deg',
                'phi_deg'])
    # est 流按最近时间配对（est 在 slam map 帧，与世界帧差刚体变换，只存位姿）
    import bisect
    est_t = [p[0] for p in est]
    prev_yaw, prev_t = None, None
    stats = []
    for k, (t, x, y, yaw) in enumerate(gt):
        if t < t1 - 0.5 or t > t1 + 30.0:   # 只看第一段跟踪（重规划前）
            continue
        fn = projD if t <= t_shift else projR
        s, e, thr, d = fn(gt[k])
        if d > 3.0:                      # 远离路径（重规划后的车早已离线）
            continue
        if prev_t is not None and t > prev_t:
            w_true = wrap(yaw - prev_yaw) / (t - prev_t)
        else:
            w_true = 0.0
        prev_yaw, prev_t = yaw, t
        jj = bisect.bisect_left(est_t, t)
        jj = min(jj, len(est) - 1)
        tr = t - t1
        w.writerow([f'{tr:.2f}', f'{s:.3f}', f'{e:+.3f}', f'{yaw:+.4f}',
                    f'{w_true:+.4f}', f'{math.degrees(thr):+.2f}',
                    f'{math.degrees(wrap(yaw - thr)):+.2f}'])
        stats.append((tr, s, e, w_true, yaw, thr, t > t_shift))
    out.close()
    print(f'写 {OUT_CSV}（{len(stats)} 帧）')

    # 摘要：e_true 峰值；关键窗口
    def peak(vals):
        return max(vals, key=abs) if vals else None
    e_trues = [st_[2] for st_ in stats]
    print(f'e_true 峰值 {peak(e_trues):+.3f} m')
    for lo, hi, tag in [(0.0, 4.33, 'D 段'), (4.33, 6.0, 'R 段头'),
                        (6.0, 12.5, 'R 段主体'), (12.5, 15.5, '收尾弧')]:
        sel = [st_ for st_ in stats if lo <= st_[1] < hi]
        if sel:
            phis = [math.degrees(wrap(st_[4] - st_[5])) for st_ in sel]
            print(f'  {tag}: {len(sel)} 帧，e_true ∈ '
                  f'[{min(s_[2] for s_ in sel):+.2f}, {max(s_[2] for s_ in sel):+.2f}]，'
                  f'φ_true ∈ [{min(phis):+.1f}°, {max(phis):+.1f}°]')


if __name__ == '__main__':
    main()
