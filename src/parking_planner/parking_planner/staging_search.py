#!/usr/bin/env python3
r"""在线自动选「倒车预备位姿」—— 全托管式选位姿，不再依赖写死的配置文件。

为什么要有这个模块
------------------
`config/staging_poses.yaml` 是**离线**用 `scripts/compute_staging.py` 算出来的：
它假设「车位数 2.5 m、邻车 4.35×1.85、过道中心线 x=±8」，再逐位验证 0 换挡。
问题是：

1. 车位里到底有没有车、有没有行人、有没有干扰车，**是运行时才知道的**。
   离线那张表只能对着「静态地图 + 假定邻车」定，一旦场上多一台车（干扰车停在
   过道边），这个位姿就不再合法 —— 真机 W5 有两次就是因为起步位姿旁边多出一台
   车（侧面间隙 0.13 m），规划器的 RS 解析解全废，退化成 26 m / 3 换挡的解。
2. 21 个垂直车位里只有 11 个在表里，其余 10 个（W0/W1/W2/W3/W6/W8/E0/E1/E4/E5/E9）
   回退到 `lot_layout` 的默认值 —— 而默认值是「车位正前方 5.5 m 的同轴位姿」，
   必须原地转 90° 才进得去，等于没做这件事。

所以：给整个停车场一个固定的入口（出生点），**每个车位的倒车位姿在运行时按
当前占用情况现场算**。算法就是离线那套的在线版：

    候选 = 车位中心 G + along·u(朝过道) + side·R·n(过道法向)
    约束 = ① 落在过道中心线上（车道图能开过去）
           ② 航向与过道单向方向一致（只能「开过车位再往回倒」）
           ③ 该位姿本身在实时栅格里是 free 的
           ④ 从它到车位中心能规划出 ≤max_sw 次换挡、余量 ≥min_clear 的解
    择优 = ① 0 换挡优先 ② 余量大优先 ③ 路径短优先

代价控制：搜索只在小范围（3 个半径 × 1 个侧向 × 2 个航向）里做，并且把
Hybrid A* 的迭代次数压到 1（走 Reeds-Shepp 解析解），单次约 0.3~0.8 s。
只在「选中车位」这一瞬间跑一次，车是停着的。
"""
import math

WRAP = lambda a: (a + math.pi) % (2 * math.pi) - math.pi   # noqa: E731


def search_staging(planner, stall, approach_yaw, lane_x, along=9.0,
                   r_list=(5.74, 6.39, 7.00), min_clear=0.30, max_sw=0,
                   band=0.35, yaw_tol=math.radians(15.0),
                   along_slack=(0.6,)):
    """返回 (score, pose, clearance, length) 或 None。

    `approach_yaw` 是过道的单向行进方向（W/E 排都是 -90°，南行）。
    `lane_x` 是过道中心线的 x（±8）；传 None 表示不校验（离线构图时用）。
    `along_slack`：第一轮只在过道中心线上找；找不到再把「沿车位轴的距离」
    放宽 ±slack（仍要落在路线能开到的地方，`_extend_to_staging` 的法向容差是
    0.6 m，所以 slack 不能超过 0.6）。

    [同学 C 本轮 §6.29] 返回的 pose 仍是**中心语义**（路线终点、对齐判据都在
    中心系）；内部校验用 planner（后轴语义）：goal 与候选位姿先沿车位轴后移
    轴距/2 换算到后轴，free/plan/min_clearance 都在后轴系里做。
    """
    gx, gy, gyaw = stall.goal_pose
    ux, uy = math.cos(gyaw), math.sin(gyaw)      # 车位轴向（指向过道）
    nx, ny = -uy, ux                             # 左法向
    ax_off = planner.wheelbase * 0.5             # 中心 → 后轴（§6.29）
    gx_r, gy_r = gx - ux * ax_off, gy - uy * ax_off  # 后轴语义的 goal
    saved_iter = planner.max_iter
    planner.max_iter = 1                          # 只求解析解，别展开栅格
    best = None
    try:
        for sl in (0.0,) + tuple(along_slack):
            for da in ((0.0,) if sl == 0.0 else (sl, -sl)):
                a = along + da
                for R in r_list:
                    for side in (1.0, -1.0):
                        # [同学 C 本轮 §6.30] 候选先按**中心语义**构建（过道带
                        # 检查、返回值都用中心），free/plan 前再沿**候选自身
                        # 朝向**换算到后轴 —— 中心→后轴的平移方向取决于位姿
                        # 朝向：候选朝过道方向、与车位轴垂直，之前沿车位轴
                        # 换算会把校验位姿挪偏 ~1.9 m（W1 在线搜索全军覆没、
                        # W4 候选的换挡数/余量评估失真的根因，diag_w1_staging.py
                        # 并排验证过）。
                        cx_ = gx + a * ux + side * R * nx
                        cy_ = gy + a * uy + side * R * ny
                        for h in (1.0, -1.0):
                            syaw = math.atan2(h * ny, h * nx)
                            if abs(WRAP(syaw - approach_yaw)) > yaw_tol:
                                continue          # 航向必须与单向过道一致
                            if lane_x is not None and abs(abs(cx_) - lane_x) > band + sl:
                                continue          # 必须落在过道中心线附近
                            sx = cx_ - math.cos(syaw) * ax_off
                            sy = cy_ - math.sin(syaw) * ax_off
                            if not planner.free(sx, sy, syaw):
                                continue          # 该位姿本身被占
                            res = planner.plan((sx, sy, syaw), (gx_r, gy_r, gyaw))
                            if not res.success or res.n_switch > max_sw:
                                continue
                            c = planner.min_clearance(list(res.path))
                            if c < min_clear:
                                continue
                            score = (2 if res.n_switch == 0 else 1,
                                     round(c, 2), -round(abs(da), 2),
                                     -round(res.length, 1))
                            # 返回值就是中心语义候选（cx_, cy_ 即中心位姿）
                            pose_c = (cx_, cy_, syaw)
                            if best is None or score > best[0]:
                                best = (score, pose_c, c, res.length)
            if best is not None:
                break                 # 中心线上能找到就不动 slack
    finally:
        planner.max_iter = saved_iter
    return best


def staging_ok(planner, pose, stall, min_clear=0.30, max_sw=0):
    """校验一个给定位姿（比如 yaml 里那份）在当前实时栅格下还合不合法。

    `pose` 与 `stall.goal_pose` 都是中心语义；内部换算到后轴再校验（§6.29）。
    """
    c, s = math.cos(pose[2]), math.sin(pose[2])
    off = planner.wheelbase * 0.5
    rear = (pose[0] - c * off, pose[1] - s * off, pose[2])
    gx, gy, gyaw = stall.goal_pose
    goal_r = (gx - math.cos(gyaw) * off, gy - math.sin(gyaw) * off, gyaw)
    if not planner.free(*rear):
        return False, None
    res = planner.plan(rear, goal_r)
    if not res.success or res.n_switch > max_sw:
        return False, None
    c = planner.min_clearance(list(res.path))
    return (c >= min_clear), res
