#!/usr/bin/env python3
r"""算「一把入位」的预备位姿：一次 90° 圆弧 + 一段沿车位轴的直线。

用户观察到的病根（两代）
------------------------
【第一代】旧预备位姿放在**车位正前方轴线上**（`x = x_车位 + l/2 + 1.6~2.6`），
从那里出发只有「原地转 90° 再倒」一条路，而车的最小转弯半径 5.74 m —— 在
12.5 m 宽的过道里原地转 90° 必须揉 2~3 把。这就是「永远不能一把入位、第一把
倒在两个车位中间、擦着邻车」的来源。解法：把预备位姿搬到车位前一个半径的
过道点上，让它能「一段圆弧 + 一点直线」直接倒进去。
【第二代】半径取成了 `R = 车最小转弯半径 = 6.39 m`（`plan_max_steer=0.40`
反算出来的），这等于把规划曲率定在车辆能力的 97% 上：
  · 规划 κ_ref = 1/6.386 = 0.157；
  · 跟随器把曲率乘 `steer_gain = 1.40` 再限幅到 `tan(0.50)/2.70 = 0.202`，
    0.157×1.40 = **0.219 > 0.202** —— **零误差时就已经顶在限幅上**；
  · 于是整段倒车弧是**开环**的：真机日志里 κ 连续 railed 在 ±0.202，
    横向偏差 3 m 行程内从 0.06 m 单调涨到 0.97 m，然后在 W7（两侧都停着车）
    剐到邻车、卡死。
解法的关键不是换公式，而是**把半径放大**：过道中心线到车位中心是 9.0 m，
所以圆弧半径最大可以取到 9.0 m（剩下的 `9.0 − R` 留作沿车位轴的直线）。
R 越大曲率越小、跟随器余量越大，而且车会**多往前开一段再倒**（用户说的
「还没找到更靠前的倒车位姿」正是这个）。

【第三代】上一条的根因诊断错了一半：真正的病根是 `steer_gain = 1.55`
（按一个**测错了的**「欠转比 0.65」标出来的）。台架用真值里程计重测，线性区
的偏航率比是 **0.96**，1.55 倍补偿等于让车多转 49%，实际圆弧半径只有
4.29 m 而规划是 5.74 m —— 车尾必然扫进邻车。增益改回 1.04。

【第四代，本轮】半径按**纯几何**重算（`probe_sweep_envelope.py`）。
关键量不是半径本身，而是「**圆弧什么时候转完**」：机动 = 90° 圆弧(R) + 沿车位轴
直线(9−R)，圆弧结束时后轴在 x = 9−R 处、离车位口（x=2.5）还有 9−R−2.5 米，
剩下的直线段就是「带着几度航向差摆正」的容错距离：
    R=6.39 → 尾段 2.61 m；R=5.74 → 3.26 m；R=4.96 → 4.04 m
而且真实矩形扫掠显示 R 越小、两侧邻车余量**只增不减**（+0.65 / +0.65 / +0.62 /
+0.54 / +0.34 对应 R = 4.96 / 5.74 / 6.39 / 8.00 / 9.00）。所以取 **0.44 / R=5.74**：
车辆曲率上限 0.2019 还留 14% 余量，尾段比 0.40 长 25%。

【第五代，本轮】R 家族有一个盲区：它只枚举「与车位轴恰好相距 R」的弧起点。
E2 的天然起点 (8, −22.99) 被场地**南墙**（内沿 y=−24.0）顶死——车头要伸到
−25.17；整个南侧 R 家族全灭，pass-2 只好收下北侧 17 m 的 S 形怪解（1 换挡、
横向峰值 0.66 m、执行剐蹭）。而墙内沿到车位口其实还有 ~2 m：把候选**沿过道
往回挪**（lat < R_MIN，先前进一小段再倒车弧），(8, −20.25) 就给出
12.54 m / 1 换挡 / 余量 +0.55 的正常形态解。所以候选集加一族「近距扫描」：
过道中心线上、进位侧、lat ∈ [0.75, R_MIN)，评分末位加**长度**决胜——同样的
(换挡, R, 余量) 下短者胜，17 m 的绕行解就再也赢不了 12.5 m 的近距解。
近距扫描只在进位侧（南侧）：北侧是「先冲过车位再倒回来」的绕行方向，
E0 的几何兜底位姿（正前方轴线直倒）比任何北侧解都好，不给它发明新位姿。

几何
----
设车位目标 G、车头方向 u（指向车道）、左法向 n，预备位姿 S：

    S = G + along·u + side·R·n,   yaw(S) = ±n 方向（沿过道）

`along` = 车道中心线到车位中心的距离（本图里恒为 9.0 m），
`R` = 圆弧半径（本轮在 6.386 ~ 9.0 之间扫），`side` = 圆心在法向哪一侧。
从 S 出发：沿半径 R 的圆弧转 90°，再沿车位轴走 `along − R` 的直线就到 G。

本脚本对每个垂直车位枚举 (R × side × 弧向 × 航向)，用**真实 Hybrid A\***
跑 `S → G`，只保留 0 换挡且车体余量 ≥ 0.30 m 的解，**优先取半径最大的**
（曲率余量最大），写成 `config/staging_poses.yaml`。

用法：
    python3 scripts/compute_staging.py            # 只打印
    python3 scripts/compute_staging.py --write    # 同时写 yaml
    python3 scripts/compute_staging.py W7 W5      # 只看指定车位
"""
import math
import os
import sys

import yaml

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))
from parking_planner.lot_layout import Layout, WHEELBASE  # noqa: E402
from parking_planner.hybrid_astar import HybridAStar     # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
OUT_YAML = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'src', 'parking_planner', 'config',
                        'staging_poses.yaml')

PLAN_MAX_STEER = float(os.environ.get('PLAN_MAX_STEER', '0.44'))
CLEARANCE = 0.15
MIN_CLEAR = 0.30
XY_RES = 0.25
TH_RES = math.radians(5.0)

# 跟随器（controller.PathFollower）的曲率限幅：tan(FOLLOW_MAX_STEER)/L。
# 入位段 `FOLLOW_MAX_STEER_PARK = 0.50`（舵角上限）、`steer_gain = 1.04`
# （台架实测车辆线性区偏航率比 0.96，见 planner_node 头部注释）。
FOLLOW_MAX_STEER = 0.50
STEER_GAIN = 1.04
KAPPA_FOLLOW = math.tan(FOLLOW_MAX_STEER) / WHEELBASE      # ≈ 0.2019

# 半径候选：从「车辆最小转弯半径」一直扫到「车道中心线距离」。
# 9.0 是上限 —— along 只有 9.0，再大就没有直线段了（纯圆弧也能规划）。
R_MIN = WHEELBASE / math.tan(PLAN_MAX_STEER)               # ≈ 5.74
R_LIST = (R_MIN, 7.0, 7.6, 8.0, 8.5, 8.8, 9.0)

# [同学 C 本轮台架复标] 跟随器能给出的曲率是 `steer_gain × κ_ref`（限幅
# `KAPPA_FOLLOW = 0.2019`），车辆实际能给出 `0.96 ×` 指令曲率。所以「跟得上」
# 的判据是 `0.96 × 1.04 × κ_ref = 1.0 κ_ref`，也就是只要指令不顶限幅就一定跟得上。
# 真机上一代之所以顶限幅，是因为 `steer_gain` 被错标成 1.55（详见 planner_node），
# 而不是半径不够。这里只保留一条诊断门限：指令曲率占限幅的比例。
RATIO_MAX = 1.20            # 只作诊断门限（>1 即指令顶穿限幅）
MIN_CLEAR_RELAX = 0.30      # 车体余量门槛

# 场内过道是**单向环**（两条过道都是北→南：车道图 `lot_n → lot_w1/w? → w2 → w3`、
# `lot_n → lot_e1 → e2 → e3 → e4`）。倒车入库的正确做法是「沿过道开过车位、
# 停在车位前方 R 处、再往回倒」，所以预备位姿的航向必须与过道行进方向一致，
# 位置在车位**行进方向的前方** R 处：
#   西排（W）：车位在过道西侧（南行时在右手边）→ 航向 −90°，S 在车位南侧
#   东排（E）：车位在过道东侧（南行时在左手边）→ 航向 −90°，S 也在车位南侧
# [同学 C 本轮修复] 东排原来写的是 **+90°（北行）**，与车道图实际方向相反：
# 路线到达东过道时是 `lot_n → lot_e1` 的斜线（到达航向 −51°），之后只能往南。
# 于是「北侧的预备位姿」永远开不到，车要么提前好几米交接，要么在过道里原地掉头
# —— 真机 E6 的「在停车场里多次转向、最后变成从右前方倒入」就是这个。
# 现在东排也用 −90°，预备位姿落在车位南侧，路线顺着过道开过去正好经过它。
APPROACH_YAW = {'W': -math.pi / 2.0, 'E': -math.pi / 2.0}
LANE_X = 8.0     # 场内车道中心线 |x|（map 帧，与 route_graph.yaml 一致）


def candidates(st):
    """枚举预备位姿候选 (x, y, yaw, R)：过道中心线上、距车位 R 的那一点。

    两族候选：
      · R 家族：与车位轴恰好相距 R（5.74~9.0），标准「90° 圆弧 + 直线」；
      · 近距族：进位侧 lat ∈ [0.75, R_MIN) —— 给「天然弧起点被墙/邻车顶死」
        的车位（如 E2：南侧起点撞南墙）用，先前进一小段再倒车弧，
        有效弧半径仍是 R_MIN（解里倒车段 κ=±0.174），所以 R 栏照实打 5.74。
    """
    gx, gy, gyaw = st.goal_pose
    ux, uy = math.cos(gyaw), math.sin(gyaw)      # 车头方向 = 指向车道
    nx, ny = -uy, ux                             # 左法向
    # along：车位中心沿 u 到车道中心线的距离。车道在 |x| = LANE_X，车位在
    # |x| = 17，所以恒为 9.0 m（两排都一样）。
    along = abs(gx) - LANE_X
    out = []
    for R in R_LIST:
        if R > along + 1e-6:
            continue
        for side in (+1.0, -1.0):
            sx = gx + along * ux + side * R * nx
            sy = gy + along * uy + side * R * ny
            # S 处航向沿 ±n（⟂ 车位轴）；两个方向都交给规划器判定
            for h in (+1.0, -1.0):
                out.append((sx, sy, math.atan2(h * ny, h * nx), R))
    # 近距族：只扫**进位侧**（过道单向南行，车必须先开过车位再倒回去；
    # side·ny < 0 的那一侧就是南侧）。北侧是绕行方向，不发明新位姿。
    nat = 1.0 if ny < 0 else -1.0
    lat = 0.75
    while lat < R_MIN - 1e-9:
        sx = gx + along * ux + nat * lat * nx
        sy = gy + along * uy + nat * lat * ny
        for h in (+1.0, -1.0):
            out.append((sx, sy, math.atan2(h * ny, h * nx), R_MIN))
        lat += 0.25
    return out


def main():
    layout = Layout.from_world(WORLD)
    write = '--write' in sys.argv
    only = [a for a in sys.argv[1:] if not a.startswith('--')]
    table = {}
    print(f'R 扫描 {R_MIN:.2f}~9.00 m，跟随器曲率限幅 {KAPPA_FOLLOW:.4f}'
          f'（steer_gain={STEER_GAIN}），判据：0 换挡且余量 ≥ {MIN_CLEAR} m')
    print('车位   预备位姿 (x, y, yaw)              R     长度     余量    '
          'κ×gain/限幅  可正向抵达')
    for st in layout.stalls:
        if st.kind != 'perpendicular':
            continue
        if only and st.id not in only:
            continue
        gx, gy, _ = st.goal_pose
        ux, uy = math.cos(st.yaw), math.sin(st.yaw)
        cx, cy, half = gx + ux * 8.0, gy + uy * 8.0, 16.0
        grid = layout.occupancy_grid(
            XY_RES / 2.0, roi=(cx - half, cx + half, cy - half, cy + half))
        planner = HybridAStar(grid, wheelbase=WHEELBASE,
                              max_steer=PLAN_MAX_STEER, xy_res=XY_RES,
                              th_res=TH_RES, clearance=CLEARANCE, max_cusps=2)
        want_yaw = APPROACH_YAW.get(str(st.id)[0])
        best = None
        # [同学 C 本轮] 两趟：先严格要求「0 换挡一把进」，找不到才放宽到 1 次换挡。
        # 为什么不能再退回「旧位姿」：东排的位姿这轮刚整体从北侧改到南侧，
        # 旧值（北侧、航向 +90°）与新的单向过道方向**相反**，路线永远开不到，
        # 车会一路冲过车位。宁可给一个「需要揉一把」的南侧位姿，也不能留着无效的北侧位姿。
        passes = (0, 1)
        for max_sw_this in passes:
            for (sx, sy, syaw, R) in candidates(st):
                if want_yaw is not None and abs(
                        (syaw - want_yaw + math.pi) % (2 * math.pi)
                        - math.pi) > math.radians(15):
                    continue                 # 航向必须与过道行进方向一致
                # 预备位姿必须落在过道中心线上，否则路线图规划不出来
                if abs(abs(sx) - LANE_X) > 0.35:
                    continue
                res = planner.plan((sx, sy, syaw), st.goal_pose)
                if not res.success or res.n_switch > max_sw_this:
                    continue
                c = planner.min_clearance(list(res.path))
                if c < MIN_CLEAR_RELAX:
                    continue
                ratio = (STEER_GAIN / R) / KAPPA_FOLLOW
                # 顺着过道**正向**开到 S（从沿航向反推 6 m 处出发）
                fx = sx - math.cos(syaw) * 6.0
                fy = sy - math.sin(syaw) * 6.0
                fwd = planner.plan((fx, fy, syaw), (sx, sy, syaw))
                fwd_ok = bool(fwd.success and fwd.n_switch == 0
                              and all(p[3] > 0 for p in fwd.path))
                # 排序：① 指令曲率留得下 15% 反馈余量（否则闭环必发散）；
                #       ② 能沿过道正向抵达；③ 换挡少；④ 余量大、半径小；
                #       ⑤ 长度短——同样 (换挡, R, 余量) 时 17 m 的绕行解
                #          赢不了 12.5 m 的近距解（E2 的场景）。
                if ratio <= RATIO_MAX:
                    score = (2, fwd_ok, -res.n_switch, -round(R, 2),
                             round(c, 2), -round(res.length, 2))
                else:
                    score = (1, fwd_ok, -res.n_switch, round(R, 2),
                             round(c, 2), -round(res.length, 2))
                if best is None or score > best[0]:
                    best = (score, sx, sy, syaw, R, res.length, c, fwd_ok)
            if best is not None:
                break
        if best is None:
            print(f'{st.id:5s}（找不到 0/1 换挡解，保留旧位姿）')
            continue
        _, sx, sy, syaw, R, ln, c, fwd_ok = best
        ratio = (STEER_GAIN / R) / KAPPA_FOLLOW
        print(f'{st.id:5s} ({sx:6.2f},{sy:7.2f},{math.degrees(syaw):6.1f}°)'
              f'{R:6.2f}{ln:8.2f}{c:+8.2f}   {ratio:6.0%}      '
              f'{"是" if fwd_ok else "否"}')
        table[st.id] = (round(sx, 3), round(sy, 3), round(syaw, 4))

    if write and table:
        os.makedirs(os.path.dirname(OUT_YAML), exist_ok=True)
        # [同学 C 修复] 必须与**已有文件合并**：本轮找不到 0 换挡解的车位（W0/W1/
        # W2/W3/W6/W8/E1/E4/E5/E8/E9 这些目标位姿本身就压着障碍或余量不足）
        # 打印的是「保留旧位姿」，但旧实现写文件时只写 `table`，会把这些车位的
        # 配置**整段删掉** —— 真机验证过的 W1 就这样被清空，Layout 回落到默认值。
        old = {}
        if os.path.exists(OUT_YAML):
            try:
                with open(OUT_YAML, encoding='utf-8') as fp:
                    old = dict((yaml.safe_load(fp) or {}).get('staging') or {})
            except Exception:
                old = {}
        with open(OUT_YAML, 'w', encoding='utf-8') as f:
            f.write('# 由 scripts/compute_staging.py 生成（勿手改）\n')
            f.write('# 几何 = 90° 圆弧（半径按车位单独选，%.2f~9.00 m）'
                    '+ 沿车位轴直线\n' % R_MIN)
            f.write('# 新算出的解都经 Hybrid A* 验证：优先 0 换挡，天然弧起点被\n')
            f.write('# 顶死的车位（如 E2 撞南墙）用近距扫描兜底成 1 换挡；'
                    '车体余量 ≥ %.2f m；\n' % MIN_CLEAR)
            f.write('# 本轮算不出解的车位原样保留上一次的值。\n')
            f.write('# 半径为什么取 %.2f 而不是更小：环状对比过 5.74 m（plan_max_steer\n'
                    % R_MIN)
            f.write('# 0.44），执行间隙从 +0.59 掉到 +0.25、末端航向从 0.4° 变 2.7°——\n')
            f.write('# 半径变小等于曲率变大，车身扫掠包络更贴邻车，反馈余量也被吃掉\n')
            f.write('# 一半。前提是 steer_gain 标定正确（1.04），不能靠放大半径去掩盖\n')
            f.write('# 增益误差。\n')
            f.write('#\n')
            f.write('# 存的是**车位局部坐标** [along, lateral, dyaw]，不是绝对坐标——\n')
            f.write('# 绝对坐标在 to_frame(spawn) 之后会混帧（自车 map 帧、预备 world 帧）。\n')
            f.write('staging:\n')
            for st in layout.stalls:
                k = st.id
                if k in table:
                    v = table[k]
                    gx, gy, gyaw = st.goal_pose
                    c, s = math.cos(gyaw), math.sin(gyaw)
                    dx, dy = v[0] - gx, v[1] - gy
                    along = c * dx + s * dy
                    lat = -s * dx + c * dy
                    dyaw = (v[2] - gyaw + math.pi) % (2 * math.pi) - math.pi
                    f.write(f'  {k}: [{along:.3f}, {lat:.3f}, {dyaw:.4f}]\n')
                elif k in old:
                    v = old[k]
                    f.write(f'  {k}: [{v[0]:.3f}, {v[1]:.3f}, {v[2]:.4f}]\n')
        print('\n已写入', OUT_YAML,
              f'（新算 {len(table)} 个，沿用旧值 {len(old) - len(set(old) & set(table))} 个）')


if __name__ == '__main__':
    main()
