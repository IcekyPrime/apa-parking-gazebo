#!/usr/bin/env python3
"""离线闭环回归：交接位姿 → 规划 → 跟车（标定过的运动学模型）→ 终点误差 + **执行轨迹真实间隙**。

不启 Gazebo，一条命令就能判断「规划/控制改了以后垂直车位还停得准不准、会不会剐」。

被控对象按实测标定：
  * 插件把 cmd_vel.angular.z 当期望偏航率，内部 delta = atan(w·L/v)（**单参数 atan**，
    v<0 时负负得正，所以 ω 会被原样复现；写成 atan2 会把倒车的 ω 整个反号）；
  * 前轮转角限幅 0.61 rad、关节速度上限 3 rad/s；
  * 实际偏航率比理想值小约 20%（实测 w=0.3 给出 0.24）。

[同学 C 修复] 原来这里只报终端位姿误差和规划器给的 `车体余量`（那是 7 个近似圆
算出来的，圆在车角处比真实车体保守、长边处又略宽松），**从来没有量过执行轨迹
上真实车体离邻车多远** —— 所以「不剐蹭」一直没有证据。现在逐步记录位姿，用
`geometry.body_gap`（真实矩形对真实矩形）算整条轨迹的最小间隙，负值即剐蹭。

用法：
    python3 scripts/verify_park_sim.py                # 全部**可用**的垂直车位
    python3 scripts/verify_park_sim.py all            # 垂直 + 平行
    python3 scripts/verify_park_sim.py W4             # 只看一个车位

需要先 source 工作区：`source ~/apa_ws/install/setup.bash`。
"""
import math
import os
import random
import sys
from collections import deque

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.controller import PathFollower                      # noqa: E402
from parking_planner.geometry import body_gap, wrap                      # noqa: E402
from parking_planner.hybrid_astar import HybridAStar, PlanResult        # noqa: E402
from parking_planner.lot_layout import (EFFECTIVE_MAX_STEER, Layout,      # noqa: E402
                                        WHEELBASE)
from parking_planner.planner_node import (MIN_SEG_LEN, SAFE_CLEAR_MARGIN,  # noqa: E402
                                          STAGE_XY_TOL, dedup_path,
                                          sample_straight)
from parking_planner.route_planner import (RouteGraph,                   # noqa: E402
                                           default_graph_path)

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds', 'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)          # 与 run_gui.sh 的默认出生点一致
XY_RES, TH_RES = 0.25, math.radians(3.0)
# 车辆实际偏航率 ÷ 指令偏航率。
#
# [同学 C 本轮台架复标 —— 这是整个入位链路上最关键的一次纠错]
# 之前的 0.65~0.80 是**从真机日志里单帧差分 yaw 反推**出来的，那个量在 20 Hz 下
# 每帧只有 0.004 rad，被定位噪声完全淹没，结论不可信。这一轮改用空世界台架
# （`scripts/bench_steer.sh` + `scripts/bench_steer.py`，真值里程计 `/odom_gt`，
# 用「弧长 + 展开航向增量」反算曲率）直接量执行器：
#     v=+0.45 m/s（入位速度）  κ=0.100/0.140/0.175/0.200 → 比值 0.98/0.97/0.96/0.96
#     v=+1.20 m/s（巡航速度）  同上                     → 比值 1.02/1.00/0.97/0.96
#     v=-0.45 m/s（倒车入位）  同上                     → 比值 0.98/0.97/0.96/0.94
# 也就是**线性区几乎没有转向不足（-3%）**，之前按 0.65 补偿（steer_gain=1.55）
# 等于让车多转 55%：实际曲率 1.55×0.96=1.49 倍于规划曲率，被限幅压成
# κ=0.2431×0.96=0.233（R=4.29 m，而规划是 R=5.74 m）——圆弧整整小了 1.45 m，
# 车尾在车位口就扫进邻车，然后停在离车位中心 2.8~3.8 m 处、间隙 0.03~0.10 m。
# 这与真机 W4/W5 的失败形态**完全吻合**，说明根因是增益标定，不是半径不够。
# 只有 κ≥0.22（δ≥28°）时比值才掉到 0.87，那是**前内轮被 0.61 rad 卡死**的
# 几何饱和（R<4.6 m 才会触发），本方案规划的 κ=0.1747 完全在饱和点以下。
UNDER = float(os.environ.get('UNDER', '0.96'))
STEER_LIMIT, STEER_RATE_LIMIT = 0.61, 3.0
# 被跟踪参考点（底盘原点）在后轴前方的距离（m）：轴距中点。
# 仿真运动学用它生成中心点的真实速度（含 ω·l_r 横向分量），见 simulate()。
REF_AHEAD = WHEELBASE / 2.0
GOAL_XY_TOL, GOAL_YAW_TOL = 0.15, math.radians(6.0)
# 与 planner_node 的 follower_park 保持一致：舵角上限 0.50、steer_gain 1.04。
# 0.50 → κ_max = tan(0.50)/2.70 = 0.2019，正好等于台架量到的「满舵实际曲率」
# 0.2015（R=4.96 m）—— 再大就是指令一个车根本给不出的曲率，只会让反馈失效。
# 规划曲率 0.1747（R=5.74）、指令 0.1747×1.04=0.182，留 11% 反馈余量。
PARK_MAX_STEER, PARK_STEER_GAIN = 0.50, float(
    os.environ.get('PARK_STEER_GAIN', '1.04'))
# 偏航率阻尼系数（与 follower_park 一致）。真机上这一项靠 IMU 提供真实偏航率；
# 离线这里用被控对象上一拍的角速度 `om` 回灌，等价于 IMU 的单步延迟。
PARK_K_RATE = float(os.environ.get('PARK_K_RATE', '0.8'))
# 入位段限速（与 follower_park 一致：0.45）
PARK_V_MAX = float(os.environ.get('PARK_V_MAX', '0.45'))
# 入位段预瞄纯跟踪的前视距离（m）。0 = 用旧的 e/φ 反馈律。
# 2.4 与 follower_park 一致：预瞄把航向增益从 1.8 降到 0.83，
# 末端航向 0.5°→0.1~0.6°，W 排执行间隙 +0.39→+0.63。
PARK_PREVIEW = float(os.environ.get('PARK_PREVIEW', '2.4'))
# 转向执行的一阶滞后时间常数（s）。按真机 φ 的「冲过头再回来」反推。
STEER_LAG = float(os.environ.get('STEER_LAG', '0.25'))
# [同学 C 本轮] **位姿测量延迟**（s）。这是离线模型与真机之间最后一个没建模的环节。
#
# **实测值 0.08 s，不是 1 s。** `scripts/pose_audit.py` 把 `/vehicle_pose` 与
# `/odom_gt` 逐帧记下来做滞后扫描（真机 E3 全程 486 s / 29.4 Hz vs 19.6 Hz）：
#     位置配准残差最小 τ=+0.10 s
#     速度互相关最大   τ=+0.10 s（r=0.996）
#     偏航率互相关最大 τ=+0.04 s（r=0.983）
#     配准后 位置 RMSE 0.045 m、P95 0.088 m、最大 0.191 m、航向 RMS 0.18°
# 三个互相独立的估计一致 → 确实是纯延迟，但只有 ~0.08 s；而且 486 s 内**没有漂移**
# （10 s 分桶的残差全程 0.02~0.09 m）。所以「slam_toolbox 阶梯式 map→odom 带来的
# 1 s 延迟」这个怀疑是**错的**，本文件原先按 1 s 建的那一版也据此收回。
#
# 建模方式仍然是纯延迟环节：控制器只能看到 `t − POSE_LAT` 时刻的位姿（历史缓冲
# 线性插值），指令作用在真实位姿上。`POSE_LAT=1.0` 仍可用来跑敏感性对照。
POSE_LAT = float(os.environ.get('POSE_LAT', '0.08'))
# 偏航率反馈是否可用（真机现状见下）。
#   1 = 有 IMU：把上一拍真实角速度回灌给控制器（旧行为，等价于 IMU 单步延迟）；
#   0 = 复刻真机现状：`/imu` 一直没数据，`_yaw_rate()` 返回 None，阻尼项关闭。
# 为什么要单独给开关：`PARK_K_RATE=0.8` 时阻尼项把曲率指令往下压，离线因此比真机
# 「更稳」。真机 IMU 没接上，这一项**根本没生效**，不建模它离线就会系统性偏乐观。
IMU_FB = os.environ.get('IMU_FB', '1') == '1'
# 航向**测量**偏差（度）：控制器看到的 yaw = 真实 yaw + 这个常数。
# 为什么需要这个量：真机 E3 首轮是「锚定预备位姿、0 换挡理想弧」，几何上起点就在
# 路径上、航向也对，可日志第一拍就是 φ=+1.4°、e=+0.09，第 3 段就 4.3°。真实几何
# 没偏而测出来的航向有 1.4° 的偏差，只能来自 `/vehicle_pose` 的航向（slam_toolbox
# 的 map→odom 是阶梯量）。这个量在日志里直接读得到，不是猜的。
POSE_YAW_BIAS = math.radians(float(os.environ.get('POSE_YAW_BIAS', '0')))
# [同学 C 新增] 定位测量噪声（复刻真机 slam 的慢漂 + 白噪声）。
# 实测（pose_audit，E2 a1）：位置残差 RMSE 0.066 m / P95 0.116 / 峰值 0.290 m，
# 且**不是白噪声**——60 s 内符号翻转 8 次（周期 ~7 s 的低频漂移，map→odom
# 阶梯修正）。白噪声会被转向滞后天然平均掉，慢漂才是把预瞄/反馈搅乱的激励。
#   POSE_DRIFT_AMP  横向慢漂幅度（OU 过程，时间常数 2 s），默认 0 = 关
#   POSE_NOISE      各向白噪声 σ（m），默认 0
#   POSE_YAW_NOISE  航向白噪声 σ（度），默认 0
# 复现真机量级：POSE_DRIFT_AMP=0.15 POSE_NOISE=0.03 POSE_YAW_NOISE=0.2
POSE_DRIFT_AMP = float(os.environ.get('POSE_DRIFT_AMP', '0'))
POSE_NOISE = float(os.environ.get('POSE_NOISE', '0'))
POSE_YAW_NOISE = math.radians(float(os.environ.get('POSE_YAW_NOISE', '0')))
# 调试：把逐拍内部量（φ / κ / e / remaining）写成 CSV，用来和真机日志逐行对照。
TRACE_CSV = os.environ.get('TRACE_CSV', '')
K_INT, INT_LIMIT = 0.0, 0.5
# 规划车体余量，与 planner_node 的 plan_clearance 参数保持一致（默认 0.15）
CLEARANCE = float(os.environ.get('PLAN_CLEARANCE', 0.15))

# 只测**目标位姿本身不压障碍**的车位（其余车位物理上停不进去，见 check_goal_gap.py）
GOAL_MIN_GAP = 0.02

# 交接位姿的“真实偏差”复现：真机上自车是在 `remaining < 1.5 m` 时停车交接的，
# 停哪儿完全取决于当场位姿，**并不等于路线的最后一个点**；而且横向还有 0.2~0.5 m
# 的跟踪残差。离线只用 route[-1] 当起点，等于把一个理想条件当成了真实条件 ——
# 这是「离线全过、真机 W4 失败」的根源。用这两个环境变量把偏差还原回来：
#   HANDOFF_BACK=1.0  HANDOFF_LAT=0.3  python3 scripts/verify_park_sim.py
HANDOFF_BACK = float(os.environ.get('HANDOFF_BACK', '0'))
HANDOFF_LAT = float(os.environ.get('HANDOFF_LAT', '0'))
# [同学 C 新增] 交接时的**航向**残差（度）。真机 W7 的交接日志是
# 「距预备位姿 0.4 m / 12°」—— 12° 的航向差足以让 Hybrid A* 把原本
# 「0 换挡的解析弧」换成「先往前挪一把、再倒」的 1 换挡解，而那条解在
# 两侧都有车的车位里跟不住（真机横向偏差 3 m 内从 0.06 涨到 0.97 m）。
HANDOFF_YAW = math.radians(float(os.environ.get('HANDOFF_YAW', '0')))

# 规划起点是否**锚定在预备位姿上**（详见 main 里的说明）。
# 1 = 锚定（几何永远是验证过的 0 换挡圆弧，自车带偏差去追）
# 0 = 从真实交接位姿规划
PLAN_AT_STAGE = os.environ.get('PLAN_AT_STAGE', '0') == '1'
# 锚定在预备位姿所允许的最大交接航向差（度），与 planner_node 的
# `anchor_yaw_tol` 同一套判据（见那里的扫参数据）。
ANCHOR_YAW_TOL = float(os.environ.get('ANCHOR_YAW_TOL', '4.0'))

# 规划栅格 ROI 半宽（m）。默认 0 = 按三点范围动态算（下限 15）。
# [同学 C 本轮] 动态 ROI 会**裁掉大半径弧**：E2 在 plan_max_steer=0.40 下
# 的 1 换挡解析解（15.8 m、κ=0.157）扫出 15 m 框，小 ROI 里解析解直接
# success=False、完整搜索退成 3 换挡 —— 与真机 42.5 m 局部窗口给出的解
# 完全不同，离线结论被 ROI 扭曲。要复现真机就用 ROI_HALF=25 以上。
ROI_HALF = float(os.environ.get('ROI_HALF', '0'))

# 入位规划用的等效前轮转角上限（rad），对应 `plan_max_steer` 参数（默认 0.42）。
# 0.42 → κ=0.165、R=6.04 m，反馈余量 18%。0.44 时 E2 南侧位姿的唯一解是满
# 曲率弧 κ=0.1747（占执行限幅 86%），换挡瞬态 0.16 m 偏差直接让指令顶死发散
# （真机 a3 行驶 101 m / INCOMPLETE）；0.40 则转弯半径超出过道半宽物理不可行。
# 扫参用法：PLAN_MAX_STEER=0.40 python3 scripts/verify_park_sim.py
PLAN_MAX_STEER = float(os.environ.get('PLAN_MAX_STEER', '0.42'))


def handoff_pose(route, back=0.0, lat=0.0, dyaw=0.0, target=None):
    """沿路线从**预备位姿**回退 back 米，再横向偏 lat、航向偏 dyaw。

    [同学 C 修正] `back` 必须相对**预备位姿**量，不能相对「路线最后一个点」：
    车道图的最后一个节点不一定落在预备位姿上（W5/E0/E3/E6/E7 的路线终点比
    预备位姿短 2.6~5.6 m）。按路线末点回退会多退出一大截，把「提前 3.2 m
    交接」模拟成「提前 6 m 交接」，结论整个反过来。
    """
    r = list(route)
    p = r[-1]
    if target is not None and dist(p, target) > 1e-3:
        # 复刻 planner_node._extend_to_staging：把路线端点**对齐到预备位姿**
        # ——「不够」就补一段直道，「冲过了」就截断。
        # ⓪ 先单独判「路线末端 → 预备位姿」（与 planner_node 的⓪同一套逻辑）
        ex, ey = target[0] - r[-1][0], target[1] - r[-1][1]
        end_gap = math.hypot(ex, ey)
        th_end = math.atan2(r[-1][1] - r[-2][1], r[-1][0] - r[-2][0]) \
            if dist(r[-1], r[-2]) > 1e-6 else r[-1][2]
        perp = abs(-math.sin(th_end) * ex + math.cos(th_end) * ey)
        along = math.cos(th_end) * ex + math.sin(th_end) * ey
        if perp <= 0.6 and 0.0 < along <= 8.0:
            r = list(r) + [tuple(target)]
            p = r[-1]
        else:
            best, acc = None, 0.0
            for i in range(len(r) - 1):
                a, b = r[i], r[i + 1]
                dx, dy = b[0] - a[0], b[1] - a[1]
                seg2 = dx * dx + dy * dy
                if seg2 < 1e-12:
                    continue
                tt = ((target[0] - a[0]) * dx + (target[1] - a[1]) * dy) / seg2
                tt = max(0.0, min(1.0, tt))
                px, py = a[0] + dx * tt, a[1] + dy * tt
                dd = math.hypot(target[0] - px, target[1] - py)
                if best is None or dd < best[0]:
                    best = (dd, i, tt, acc + math.sqrt(seg2) * tt)
                acc += math.sqrt(seg2)
            if best is not None:
                d_lat, i, tt, s_star = best
                over = acc - s_star
                # 与 planner_node._extend_to_staging 同一套判据（含「路线还没开到」
                # 时把法向分量单独拎出来那条修复）
                d_perp = d_lat
                if i >= len(r) - 2 and tt > 0.985 and len(r) >= 2:
                    th_e = math.atan2(r[-1][1] - r[-2][1],
                                      r[-1][0] - r[-2][0])
                    gx, gy = target[0] - r[-1][0], target[1] - r[-1][1]
                    d_perp = abs(-math.sin(th_e) * gx + math.cos(th_e) * gy)
                if d_perp <= 0.6 and over <= 6.0:
                    seg_yaw = math.atan2(r[i + 1][1] - r[i][1],
                                         r[i + 1][0] - r[i][0])
                    trunc_ok = abs(wrap(seg_yaw - target[2])) <= math.radians(60.0)
                    if over > 0.15 and trunc_ok:      # 冲过了且方向一致：截断
                        head = list(r[:i + 1])
                        if tt > 1e-6:
                            ya, yb = r[i][2], r[i + 1][2]
                            head.append((r[i][0] + (r[i + 1][0] - r[i][0]) * tt,
                                         r[i][1] + (r[i + 1][1] - r[i][1]) * tt,
                                         wrap(ya + wrap(yb - ya) * tt)))
                        r = head + [tuple(target)]
                    else:                            # 还差 / 方向不一致：顺着车头补
                        dir_ok = abs(wrap(math.atan2(target[1] - p[1],
                                                     target[0] - p[0])
                                          - p[2])) <= math.radians(20.0)
                        if dir_ok:
                            r = r + [tuple(target)]
                    p = r[-1]
    if back > 0.0 and len(r) > 2:
        # [同学 C 修正] 必须**在段内插值**，不能只跳到上一个采样点。
        # 旧写法只累加整段、返回 `r[i]`：当最后补进来的那一段有 3.1 m 时，
        # 「退 0.4 m」实际退成了 3.1 m（E0/E6 因此被判成「没锚定」而全挂）。
        acc, i = 0.0, len(r) - 1
        while i > 1 and acc < back:
            acc += dist(r[i], r[i - 1])
            i -= 1
        seg = dist(r[i + 1], r[i]) if i + 1 < len(r) else 0.0
        if seg > 1e-9 and acc > back:
            t = (acc - back) / seg          # 从 r[i] 往 r[i+1] 走 t 的比例
            t = max(0.0, min(1.0, t))
            a, b = r[i], r[i + 1]
            p = (a[0] + (b[0] - a[0]) * t,
                 a[1] + (b[1] - a[1]) * t,
                 wrap(a[2] + wrap(b[2] - a[2]) * t))
        else:
            p = r[i]
    if abs(lat) > 1e-6:
        p = (p[0] - math.sin(p[2]) * lat, p[1] + math.cos(p[2]) * lat, p[2])
    if abs(dyaw) > 1e-9:
        p = (p[0], p[1], wrap(p[2] + dyaw))
    return p


def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def rear(p):
    """中心位姿 → 后轴位姿（§6.29：hybrid_astar 自本轮起是后轴参考）。"""
    c, s = math.cos(p[2]), math.sin(p[2])
    return (p[0] - c * REF_AHEAD, p[1] - s * REF_AHEAD, p[2])


def straight_ok(planner, p0, p1):
    n = max(2, int(dist(p0, p1) / 0.10))
    for i in range(n + 1):
        t = i / n
        if not planner.free(p0[0] + (p1[0] - p0[0]) * t,
                            p0[1] + (p1[1] - p0[1]) * t, p0[2]):
            return False
    return True


def delayed(hist, tq):
    """从历史缓冲里取 `tq` 时刻的 (pose, omega)，线性插值。

    `hist` 元素为 (t, x, y, yaw, omega)，按时间递增。`tq` 早于最老的一帧时
    退化成最老的一帧（起步阶段延迟等于「车还停在起点」——这正是真机的表现：
    刚起步那一秒控制器看到的还是起点）。
    """
    if tq <= hist[0][0]:
        h = hist[0]
        return (h[1], h[2], h[3]), h[4]
    for k in range(len(hist) - 1, 0, -1):
        t1 = hist[k][0]
        if t1 <= tq:
            a, b = hist[k - 1], hist[k]
            dt_ = t1 - a[0]
            r = 0.0 if dt_ < 1e-9 else (tq - a[0]) / dt_
            return ((a[1] + (b[1] - a[1]) * r,
                     a[2] + (b[2] - a[2]) * r,
                     wrap(a[3] + wrap(b[3] - a[3]) * r)),
                    a[4] + (b[4] - a[4]) * r)
    h = hist[-1]
    return (h[1], h[2], h[3]), h[4]


def simulate(path, start, dt=0.05, tmax=150.0):
    """跑完一条路径，返回 (终点位姿, 用时, 横向峰值(测量), 横向峰值(真实),
    换挡次数, cmd, 轨迹)。

    [同学 C 本轮] 加入**位姿测量延迟** `POSE_LAT`（默认 1.0 s，见该常量处的推导）。
    控制器只拿到 `t − POSE_LAT` 的位姿，指令作用在真实位姿上 —— 回路里多一个纯
    延迟环节。同时区分两个「横向峰值」：

    * `maxlat`（测量）：控制器自己看到的那条 e_lat，**与真机日志「横向偏差」
      同一个量**，用来和真机日志对照；
    * `maxlat_true`：真实车体到路径的垂距，用来判断「是车真的偏了」还是
      「只是控制器看到的值滞后」。

    两者之差就是延迟造成的「盲走」量，是这一轮要复现 0.86 m 的直接指标。
    """
    f = PathFollower(wheelbase=WHEELBASE, max_steer=PARK_MAX_STEER,
                     v_max=PARK_V_MAX,
                     steer_gain=PARK_STEER_GAIN,
                     k_rate=PARK_K_RATE, steer_rate=0.0,
                     preview=PARK_PREVIEW, tail_anchor=False,
                     ld_floor=float(os.environ.get('PARK_LD_FLOOR', '1.2')),
                     goal_xy_tol=GOAL_XY_TOL, goal_yaw_tol=GOAL_YAW_TOL,
                     k_int=K_INT, int_limit=INT_LIMIT,     # 与 follower_park 一致
                     ref_ahead=0.0)    # §6.29：后轴参考（与 follower_park 一致）
    f.set_path(path)
    x, y, th = start
    delta, t, maxlat, shifts = 0.0, 0.0, 0.0, 0
    maxlat_true = 0.0
    d_lag = 0.0
    om_prev = 0.0            # 上一拍的实测角速度（等价于 IMU 信号）
    prev_gear, cmd = path[0][3], None
    traj = [(x, y, th)]
    trace = []               # TRACE_CSV 用：逐拍内部量，供与真机日志逐行对照
    # 历史缓冲：`POSE_LAT` 秒的位姿要留着给控制器「回顾」
    hist = deque([(0.0, x, y, th, 0.0)],
                 maxlen=max(40, int(POSE_LAT / max(dt, 1e-3)) + 8))
    # 定位噪声状态：OU 慢漂（世界系两轴独立，时间常数 2 s）+ 白噪声
    rng = random.Random(int(os.environ.get('POSE_SEED', '7')))
    drift_x = drift_y = 0.0
    while t < tmax:
        # 控制器输入的位姿（延迟版）与真实位姿分离
        pose_seen, om_seen = delayed(hist, t - POSE_LAT) if POSE_LAT > 1e-6 \
            else ((x, y, th), om_prev)
        if POSE_DRIFT_AMP > 0.0:
            tau = 2.0
            kick = POSE_DRIFT_AMP * math.sqrt(2.0 * dt / tau)
            drift_x += (-drift_x / tau) * dt + kick * rng.gauss(0.0, 1.0)
            drift_y += (-drift_y / tau) * dt + kick * rng.gauss(0.0, 1.0)
            pose_seen = (pose_seen[0] + drift_x, pose_seen[1] + drift_y,
                         pose_seen[2])
        if POSE_NOISE > 0.0 or POSE_YAW_NOISE > 0.0:
            pose_seen = (pose_seen[0] + POSE_NOISE * rng.gauss(0.0, 1.0),
                         pose_seen[1] + POSE_NOISE * rng.gauss(0.0, 1.0),
                         wrap(pose_seen[2] + POSE_YAW_NOISE * rng.gauss(0.0, 1.0)))
        if POSE_YAW_BIAS:
            pose_seen = (pose_seen[0], pose_seen[1],
                         wrap(pose_seen[2] + POSE_YAW_BIAS))
        cmd = f.update(pose_seen, dt,
                       yaw_rate=(om_seen if IMU_FB else None))
        if cmd.gear != prev_gear:
            shifts += 1
            prev_gear = cmd.gear
        maxlat = max(maxlat, cmd.lateral_error)
        # [§6.29] 路径是后轴轨迹，真实偏差也要在**后轴参考点**上量（中心点对
        # 后轴路径的垂距会把弧上的 β 混进来，量纲就错了）。
        proj_true = f._project(x - math.cos(th) * REF_AHEAD,
                               y - math.sin(th) * REF_AHEAD)
        maxlat_true = max(maxlat_true, abs(proj_true[2]))
        if TRACE_CSV:
            d = f.dbg
            trace.append(f'{t:.2f},{x:.3f},{y:.3f},{th:.4f},'
                         f'{cmd.lateral_error:.3f},{d.get("phi", 0.0):+.4f},'
                         f'{d.get("kappa", 0.0):+.4f},{cmd.gear},{cmd.remaining:.3f},'
                         f'{d.get("la_d", 0.0):.3f},{d.get("k_pp", 0.0) or 0.0:+.4f},'
                         f'{d.get("e", 0.0):+.3f},{d.get("thr", 0.0):+.4f},'
                         f'{d.get("k_ref", 0.0):+.4f},{d.get("seg", 0)},'
                         f'{proj_true[2]:+.3f},{wrap(th - proj_true[3]):+.4f},'
                         f'{1 if d.get("flip", False) else 0}')
        if cmd.done or cmd.stuck:
            break
        v, w = cmd.v, cmd.omega
        d = math.atan(w * WHEELBASE / v) if abs(v) > 1e-3 else delta
        d = max(-STEER_LIMIT, min(STEER_LIMIT, d))
        # 一阶转向滞后 + 速率限幅：真机日志里倒车弧的 φ 是「先冲过头再回来」，
        # 这是滞后而非稳态增益误差，不建模它就永远复现不出真机的漂移。
        # STEER_LAG<=0 表示关掉滞后（只留速率限幅）。
        if STEER_LAG > 1e-6:
            d_lag += (d - d_lag) * min(1.0, dt / STEER_LAG)
        else:
            d_lag = d
        step = STEER_RATE_LIMIT * dt
        delta += max(-step, min(step, d_lag - delta))
        om = v * math.tan(delta) / WHEELBASE * UNDER
        om_prev = om
        th = wrap(th + om * dt)
        # [同学 C 本轮 §6.28] **真实中心点运动学**。被跟踪位姿是底盘原点（轴距
        # 中点，后轴前方 l_r=WHEELBASE/2），弧线运动时它的速度**不在航向上**：
        # 刚体运动学 v_center = v_rear + ωẑ×(l_r,0) = (v, ω·l_r)（车体系，v 带符号），
        # 速度方向偏离航向 β = atan(κ·l_r) —— κ=0.165 的弧上 β≈12.6°。真机 E2
        # D 段实测（/odom_gt 速度方向 − 航向）稳定在 -12°~-15°，与
        # atan(0.165×1.35) 精确吻合：这不是轮胎侧滑，是几何必然。
        # 旧点模型把这一项丢了，离线「完美跟弧」而真机 e 以 0.17/m 单调发散
        # （φ=+2° 时速度仍偏切线 11°）；历史上 W7「倒车弧同心圆、φ→0 但 e 线性
        # 涨到 1.05 m」也是同一根因。控制器侧的对应修复是 β 补偿（controller.py
        # 的 ref_ahead）。
        x += (v * math.cos(th) - om * REF_AHEAD * math.sin(th)) * dt
        y += (v * math.sin(th) + om * REF_AHEAD * math.cos(th)) * dt
        t += dt
        traj.append((x, y, th))
        hist.append((t, x, y, th, om))
    if TRACE_CSV:
        with open(TRACE_CSV, 'w') as fp:
            fp.write('t,x,y,yaw,e_meas,phi,kappa,gear,remaining,la_d,k_pp,'
                     'e_sig,thr,kref,seg,e_true,phi_true,flip\n')
            fp.write('\n'.join(trace) + '\n')
    return (x, y, th), t, maxlat, maxlat_true, shifts, cmd, traj


def traj_gap(traj, obstacles, half_l=2.175, half_w=0.925):
    """执行轨迹上**真实车体矩形**的最小间隙（m）以及出现时的位姿。"""
    best, at = float('inf'), None
    for p in traj:
        g = body_gap(p, obstacles, half_l=half_l, half_w=half_w)
        if g < best:
            best, at = g, p
    return best, at


def strip_micro_head(planner, res, goal):
    """复刻 planner_node._strip_micro_head：起步小段与换挡点重规划弧**拼接**。

    原实现「直接替换成从换挡点出发的新路径」会让路径起点悬在车前方 0.3~0.6 m
    处（W9 第一拍 φ=−14.3° 的来源）；拼接保留原起步头，起点仍是车的位姿。
    """
    if not res.success:
        return res
    path = list(res.path)
    i = next((k for k in range(1, len(path)) if path[k][3] != path[0][3]), None)
    if i is None:
        return res
    leg = sum(dist(a, b) for a, b in zip(path[:i], path[1:i + 1]))
    if leg >= MIN_SEG_LEN:
        return res
    r = planner.plan(path[i][:3], goal)
    if r.success:
        merged = dedup_path(path[:i + 1] + list(r.path))
        if len(merged) >= 2:
            n_sw = sum(1 for a, b in zip(merged, merged[1:]) if a[3] != b[3])
            # 门禁用拼接后的实际换挡数（与 planner_node 同一修复）：拼接点可能
            # 再引入一次换挡（R→D→R 抖动），旧写法检查 r.n_switch 会放行劣化解。
            if n_sw <= res.n_switch:
                rev = sum(dist(a, b) for a, b in zip(merged, merged[1:])
                          if b[3] < 0)
                total = leg + r.length
                return PlanResult(True, merged, cost=total, reason='拼接',
                                  used_rs=True, length=total,
                                  reverse_length=rev, n_switch=n_sw)
    return res


def plan_path(planner, hand, stall, min_clear=0.30):
    """复刻 planner_node._plan 的选解规则，返回 (res, path, used_stage)。

    [同学 C 修复] 选解从「(换挡次数, 长度) 字典序」改成**余量优先**：
    0 换挡只说明少揉一把，不说明这条线离邻车够远（见 compare_handoff.py 的实测：
    W7 直连 0 换挡余量 0.168 → 执行后间隙 0.00 m 剐蹭；两段式 1 换挡余量 0.550
    → 执行后 0.45 m）。

    [同学 C 本轮 §6.29] hand/goal/staging 全部换算到**后轴**再进规划器
    （与 planner_node._rear 同一规则）；`near_stage` 等物理判据保持中心语义。
    """
    hand_r = rear(hand)
    goal_r = rear(stall.goal_pose)
    stage_r = rear(stall.staging_pose)
    direct = strip_micro_head(planner, planner.plan(hand_r, goal_r), goal_r)
    if stall.kind != 'perpendicular':
        # [本轮修复] 与 planner_node `_plan` 的平行位分支严格对齐（三步走）：
        #   1) 最小搜索预算的直连解析解 —— **成功就直接采用**（真机逻辑）；
        #   2) 失败才试「开到预备位姿 + 从预备位姿揉库」两段式；
        #   3) 再失败才退回完整搜索。
        saved = planner.max_iter
        planner.max_iter = 1
        quick = planner.plan(hand_r, goal_r)
        planner.max_iter = saved
        if quick.success:
            return quick, list(quick.path), False
        a = planner.plan(hand_r, stage_r)
        b = planner.plan(stage_r, goal_r) if a.success else None
        if b is not None and b.success:
            res = a
            res.success = True
            res.length = a.length + b.length
            res.reverse_length = a.reverse_length + b.reverse_length
            res.n_switch = a.n_switch + b.n_switch
            res.reason = '两段式'
            return res, list(a.path) + list(b.path)[1:], True
        return quick, list(quick.path), False

    near_stage = dist(hand, stall.staging_pose) < 0.8
    if ((not direct.success) or direct.n_switch >= 1) and not near_stage:
        r2 = planner.plan(hand_r, stage_r)
        tail_res = planner.plan(stage_r, goal_r)
        tail = None
        if tail_res.success and tail_res.n_switch == 0:
            tail = list(tail_res.path)
        elif straight_ok(planner, stage_r, goal_r):
            tail = sample_straight(stage_r, goal_r, 0.10)
            tail_res = None
        return _choose(planner, direct, r2, tail, tail_res, stall, min_clear)
    return direct, list(direct.path), False


def _choose(planner, direct, r2, tail, tail_res, stall, min_clear):
    if r2.success and tail is not None:
        # 两段拼接后必须去重（与 planner_node 的 `dedup_path` 同一套规则）：
        # 第一段终点 = 尾巴起点，再加上 RS 路径自带的重复末点，会留下长度 0 的
        # 线段，跟车器的垂足卡在原地不动（实测终点误差 12.47 m、车只走 0.14 m）。
        path2 = dedup_path(list(r2.path) + tail)
        n_sw = sum(1 for a, b in zip(path2, path2[1:]) if a[3] != b[3])
        rev = sum(dist(a, b) for a, b in zip(path2, path2[1:]) if b[3] < 0)
        total = sum(dist(a, b) for a, b in zip(path2, path2[1:]))
        if not direct.success:
            return PlanResult(True, path2, cost=total, reason='两段式',
                              used_rs=True, length=total,
                              reverse_length=rev, n_switch=n_sw), path2, True
        c_d = planner.min_clearance(list(direct.path))
        c_2 = planner.min_clearance(path2)
        ok_d, ok_2 = c_d >= min_clear, c_2 >= min_clear
        sw2 = r2.n_switch + (tail_res.n_switch if tail_res else 1)
        len2 = r2.length + sum(dist(a, b) for a, b in zip(tail, tail[1:]))
        # 与 planner_node 的选解规则保持一致：都够安全时，余量差 ≥0.15 m 就按
        # 余量选（多揉一把换更大的执行余量是值得的），否则才比换挡/长度。
        if ok_2 and not ok_d:
            take = True
        elif ok_d and not ok_2:
            take = False
        elif ok_d and ok_2:
            if c_2 >= c_d + SAFE_CLEAR_MARGIN:
                take = True
            elif c_d >= c_2 + SAFE_CLEAR_MARGIN:
                take = False
            else:
                take = (sw2, len2) < (direct.n_switch, direct.length)
        else:
            take = c_2 > c_d
        if take:
            # [同学 C 修复] 元数据必须按**拼接后的完整路径**重算：旧写法直接把
            # r2（开到预备位姿那一段）当成整个解返回，表格里 W9 显示成
            # 「路径长 1.5 m / 换挡 0」，而实际执行的是 13.8 m / 1 换挡 ——
            # 回归表读起来完全是另一个故事。
            return PlanResult(True, path2, cost=total, reason='两段式',
                              used_rs=True, length=total,
                              reverse_length=rev, n_switch=n_sw), path2, True
    return direct, list(direct.path), False


def main():
    arg = sys.argv[1] if len(sys.argv) > 1 else None
    layout = Layout.from_world(WORLD).to_frame(SPAWN)
    graph = RouteGraph.load(default_graph_path()).to_frame(SPAWN)

    # 候选：默认只测「目标位姿本身不压障碍」的垂直车位（含 W0..W9 / E0..E9 全部，
    # 不再像以前那样只挑单号 —— 用户的 W4 就落在被跳过的那些里）
    ids = []
    if arg and arg not in ('all',):
        ids = [arg]
    else:
        for st in layout.stalls:
            if st.kind != 'perpendicular' and arg != 'all':
                continue
            if body_gap(st.goal_pose, layout.obstacles) <= GOAL_MIN_GAP:
                continue          # 目标位姿本身压着车，物理上停不进去
            ids.append(st.id)

    print(f'{"车位":5s} {"类型":9s} {"规划":7s} {"路径长":7s} {"两段":5s} {"换挡":4s} '
          f'{"横向峰值":8s} {"真实横向":8s} {"终点误差":9s} {"航向":7s} '
          f'{"执行最小间隙":12s} 判定')
    if POSE_LAT > 1e-6:
        print(f'  （位姿测量延迟 {POSE_LAT:.2f} s{"，IMU 偏航率可用" if IMU_FB else "，IMU 缺席→阻尼关闭"}）')
    ok = total = scraped = 0
    for tid in ids:
        st = layout.find_stall(tid)
        route = graph.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
        if not route:
            print(f'{tid:5s} 路线规划失败')
            continue
        hand = handoff_pose(route, HANDOFF_BACK, HANDOFF_LAT, HANDOFF_YAW,
                            target=st.staging_pose)
        pts = [hand, st.goal_pose, st.staging_pose]
        cx = sum(p[0] for p in pts) / 3.0
        cy = sum(p[1] for p in pts) / 3.0
        half = ROI_HALF if ROI_HALF > 0 else max(
            15.0, max(dist(p, (cx, cy)) for p in pts) + 6.0)
        grid = layout.occupancy_grid(XY_RES / 2.0,
                                     roi=(cx - half, cx + half, cy - half, cy + half))
        planner = HybridAStar(grid, wheelbase=WHEELBASE, max_steer=PLAN_MAX_STEER,
                              xy_res=XY_RES, th_res=TH_RES,
                              # 与 planner_node 一致：只有垂直车位留车体余量
                              clearance=CLEARANCE if st.kind == 'perpendicular' else 0.0,
                              max_cusps=2 if st.kind == 'perpendicular' else 3,
                              # [同学 C §6.26] 与 planner_node 一致：解析解按真实
                              # 车体余量挑（W5/W7/E3/E8 的最短解只有 +0.068 m）。
                              # [本轮修复] 平行位也**不再**给 0.0：那等于「只挑最短」，
                              # 真机 P1/P3 拿到的 +0.05/+0.18 m 解执行后间隙 0.00 m。
                              clear_target=0.30 if st.kind != 'parallel' else 0.45)
        # [同学 C 新增] 「机动锚在预备位姿上」：规划起点用**预备位姿**（几何永远
        # 是那条验证过的 0 换挡圆弧），但车还是从真实交接位姿出发，靠反馈去追。
        # 真机 W7 的交接位姿总差 0.4~0.5 m / 8~10°，从那里规划出来的弧余量会从
        # +0.55 掉到 +0.18 —— 差别就在这 0.37 m 上。
        plan_start = hand
        # [同学 C 本轮] 与 planner_node 的 `anchor_yaw_tol` 保持同一套判据：
        # 交接航向差 ≤4° 才锚定（几何是验证过的 0 换挡弧），
        # 更大就改从真实位姿规划（多 1 次换挡，但余量从 0 变成 +0.4）。
        # [同学 C §6.30] 直线解车位（staging 与 goal 同航向共线，如 E0 条目
        # [5.5,0,0]）放宽到 10°：直线 κ=0 无曲率余量约束，与 planner_node
        # 的 `_straight_stage` 同一判据。
        _gdx = st.goal_pose[0] - st.staging_pose[0]
        _gdy = st.goal_pose[1] - st.staging_pose[1]
        _straight_stage = (
            abs(wrap(st.staging_pose[2] - st.goal_pose[2])) < math.radians(1.0)
            and abs(-math.sin(st.staging_pose[2]) * _gdx
                    + math.cos(st.staging_pose[2]) * _gdy) < 0.1)
        _anchor_tol = 10.0 if _straight_stage else ANCHOR_YAW_TOL
        # [同学 C §6.31] 与 planner_node 同一套「沿 / 横分解」位置判据（各 0.30 m）。
        # 旧写法 `dist(hand, staging) < 1.2` 会把「沿过道差 1.13 m、航向 0°」当真，
        # 锚定后跟踪首拍就带 1.13 m 初始横向误差（真机 W7 间隙 0.00 m 的根因）。
        _dx_ = st.staging_pose[0] - hand[0]
        _dy_ = st.staging_pose[1] - hand[1]
        _c_, _s_ = math.cos(st.staging_pose[2]), math.sin(st.staging_pose[2])
        _st_along = _c_ * _dx_ + _s_ * _dy_
        _st_lat = -_s_ * _dx_ + _c_ * _dy_
        if (PLAN_AT_STAGE
                and abs(_st_along) <= STAGE_XY_TOL
                and abs(_st_lat) <= STAGE_XY_TOL
                and abs(wrap(hand[2] - st.staging_pose[2]))
                <= math.radians(_anchor_tol)):
            plan_start = tuple(st.staging_pose)
        if os.environ.get('DIAG_STAGE'):
            d_hs = dist(hand, st.staging_pose)
            y_hs = math.degrees(abs(wrap(hand[2] - st.staging_pose[2])))
            print(f'  [{tid} diag] route_len={sum(dist(a, b) for a, b in zip(route, route[1:])):.1f} '
                  f'route_end=({route[-1][0]:.2f},{route[-1][1]:.2f},{math.degrees(route[-1][2]):.0f}°) '
                  f'hand=({hand[0]:.2f},{hand[1]:.2f},{math.degrees(hand[2]):.0f}°) '
                  f'stage=({st.staging_pose[0]:.2f},{st.staging_pose[1]:.2f},'
                  f'{math.degrees(st.staging_pose[2]):.0f}°) '
                  f'd={d_hs:.2f}（沿 {_st_along:+.2f} / 横 {_st_lat:+.2f}） '
                  f'dyaw={y_hs:.1f}° '
                  f'anchor={"Y" if plan_start != hand else "N"} '
                  f'plan_from=({plan_start[0]:.2f},{plan_start[1]:.2f})')
        res, path, used = plan_path(planner, plan_start, st)
        if not res.success:
            print(f'{tid:5s} {st.kind:9s} 失败 {res.reason}')
            continue
        total += 1
        final, t, maxlat, maxlat_true, shifts, cmd, traj = simulate(path, hand)
        e = dist(final, st.goal_pose)
        ey = math.degrees(abs(wrap(final[2] - st.goal_pose[2])))
        gap, at = traj_gap(traj, layout.obstacles)
        good = e <= 0.35 and ey <= 9.0 and gap > 0.0
        ok += 1 if good else 0
        if gap <= 0.0:
            scraped += 1
        note = '剐蹭!' if gap <= 0.0 else ('' if good else
                                          ('stuck' if cmd.stuck else '超差'))
        print(f'{tid:5s} {st.kind:9s} {res.reason:7s} {res.length:7.1f} {str(used):5s} '
              f'{shifts:4d} {maxlat:8.2f} {maxlat_true:8.2f} {e:8.2f} {ey:6.1f}° '
              f'{gap:+11.2f} m {note}')
    if total:
        print(f'\n达标 {ok}/{total}（判据：终点 ≤0.35 m / ≤9°，且执行轨迹全程车体间隙 >0；'
              f'剐蹭 {scraped} 个）')


if __name__ == '__main__':
    main()
