#!/usr/bin/env python3
"""泊车规划节点：全局路线 → 选位 → 局部规划 → 跟踪 → 入位。

状态机
------
WAIT_DATA → SELECT_SPOT → ROUTE（远距离时先按车道图开过去）
                        → PLAN（局部 Hybrid A*）→ FOLLOW → DONE / FAILED

关键点
------
1. 自车可以在主路任意位置生成，因此布局在启动时按 `spawn_*` 参数从 world 帧
   变换到 map 帧；
2. 目标车位可以是场内垂直/平行车位，也可以是四条道路边上的平行车位，
   用 `goal_stall` 指定（如 W5 / P1 / SW3 / B2 / E9），留空则自动选择；
3. 平行车位不要求一把入位，Hybrid A* 允许最多 3 次换挡（多点揉库）。
"""

import json
import math
import os
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Point, PointStamped, PoseArray, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from .controller import PathFollower
from .geometry import transform_point, wrap
from .hybrid_astar import HybridAStar, PlanResult
from .lot_layout import (EFFECTIVE_MAX_STEER, FOLLOW_MAX_STEER, Layout,
                         WHEELBASE)
from .route_planner import RouteGraph, default_graph_path
from .spot_detector import SpotDetector
from .staging_search import search_staging

# [同学 C 本轮台架复标 —— 入位链路上最关键的一次纠错]
# 之前这里写的是「实测偏航率只有指令的 0.65~0.70，所以增益取 1.55」。那个 0.65
# 是**从真机日志里单帧差分 yaw 反推**出来的：20 Hz 下每帧航向只变 0.004 rad，
# 完全被定位噪声淹没，根本量不出执行器特性。本轮改用空世界台架
# （`scripts/bench_steer.py`，真值里程计 `/odom_gt`，「弧长 + 展开航向增量」反算）：
#     v=+0.45 / +1.20 / -0.45 m/s，κ=0.100/0.140/0.175/0.200
#     → 比值 0.98|1.02|0.98 / 0.97|1.00|0.97 / **0.96|0.97|0.96** / 0.96|0.96|0.94
# 即线性区几乎不欠转（-3%），1.55 倍的补偿等于让车多转 55%：
# 实际曲率 = 1.55×0.96 = 1.49 倍规划曲率 → 被限幅压成 κ=0.233（R=4.29 m），
# 而规划是 R=5.74 m，圆弧小了 1.45 m —— 车尾在车位口扫进邻车，然后停在离车位
# 中心 2.8~3.8 m、间隙 0.03~0.10 m。这正是真机 W4/W5 的失败形态。
# 所以：`steer_gain` 回到 1.04（补掉那 4%），舵角上限回到 0.50 ——
# κ_max = tan(0.50)/2.70 = 0.2019，恰等于台架量到的满舵实际曲率 0.2015
# （R=4.96 m），再高的指令车根本给不出，只会让反馈闭环失效。
# 只有 κ≥0.22（δ≥28°）时比值才掉到 0.87，那是**前内轮被关节 0.61 卡死**的几何
# 饱和（R<4.6 m 才触发），本方案的 0.1747 离饱和点还有 13%。
FOLLOW_MAX_STEER_PARK = 0.50
PARK_STEER_GAIN = 1.04

# 分级规划的分工边界
# ---------------
# 全局路线（车道图 + 圆弧）负责「沿车道开到目标车位所在的车道」，最后几米
# （含与车道正交的 90° 入位转向）交给 Hybrid A*。因为车道折线再圆滑也只是圆弧
# 拼接，车位进出方向与车道正交时末端必然是个急弯；Hybrid A* 带运动学约束和
# 碰撞检查，能自己切角、自己多点揉库。
# 所以路线终点就是交接点：路线走完才切局部规划，不再提前若干米切 —— 否则车会
# 停在车位正前方，还得原地掉头。
ROUTE_THRESHOLD = 14.0     # 比这更近就直接做局部规划，不必再走全局路线
ROUTE_BAD_LAT = 2.0        # 路线跟踪横向偏差超过该值即提前交接（安全网）
ROUTE_BAD_HOLD = 2.5       # 且持续这么多秒，避免偶发抖动

# 入位触发的「到点」判据：必须先**准确停在预备位姿**才允许切局部规划。
# 实测只按「路线剩余 < 2.5 m」交接时，自车经常在离预备位姿 7~16 m 的地方
# 就交了班，Hybrid A* 只能凭空多规划 12~14 m、换挡 1~2 次的「两段式」路径，
# 把起始偏差一路放大成终点偏差（同一车位有时 0.32 m、有时 1.8 m）。
# 现在要求位置 ≤1.0 m、航向 ≤10° 才入位，不够就继续沿路线对齐。
DOCK_AT_STAGE_XY = 1.0
DOCK_AT_STAGE_YAW = math.radians(10.0)
# [同学 C §6.31] 「自车就在预备位姿上」的位置门限：**沿 / 横**分开判，各 0.30 m。
# 旧写法是欧氏距离 < 1.2 m，真机 W7 在航向完全对齐、只差 1.13 m 纵向时也被判
# 「已在容差内」→ 锚定到预备位姿 → 跟踪首拍 e=-0.69 m → 倒车弧扫掉邻车间隙。
# 真机 9 个通过车位的实际交接偏差是 0.11~0.16 m（W4/W5/W7 之外的 8 个车位），
# 0.30 m 留了一倍余量，不会把正常交接也推去 ALIGN。
STAGE_XY_TOL = 0.30
DOCK_AT_STAGE_RETRY = 4    # 对齐重规划次数上限（保证一定会往下走，不死等）
DOCK_AT_STAGE_PERIOD = 1.5  # 两次对齐重规划的最小间隔（秒）
# ALIGN 状态（真正的对齐机动）的超时与到点判据
ALIGN_TIMEOUT = 45.0
# 到点判据。离线扫过（`scripts/verify_align_sim.py`，且每段注入 0.12 m / 1.5° 的
# 真实跟踪残差）：位置 0.30 m / 航向 12° 就够，因为真正决定入位精度的是**入位
# 那条直线**，ALIGN 的残差会作为入位规划的起点误差被吃掉（实测残差 0.26~0.43 m /
# 1~4° 时入位仍是 0 换挡、终点 0.14 m）。
ALIGN_XY_TOL = 0.30
ALIGN_YAW_TOL = math.radians(12.0)
ALIGN_HORIZON = 2.5       # 分段滚动执行：每段执行弧长（m）
ALIGN_REPLAN_MAX = 10     # 分段重规划次数上限
# 为什么是 10 而不是 6：离线扫（`verify_align_sim.py`，每段注入 0.12 m / 1.5° 的
# 真实跟踪残差）时，11 个车位里有 6 个要用满 **10 段**才收敛；而真机把上限写成 6
# 后，W1/W4/W5 全部在第 6 段被「分段用尽」截断，然后从离预备位姿还有 4~5 m 的
# 位姿去入位，必然失败（真机实测 W1 5.48 m、W5 5.07 m、W4 3.93 m）。
# 发散判据：ALIGN 里最怕「越走越远」。但要注意 **多点机动天生先远后近**——
# 实测 W4 的对齐解是「先前进 2.5 m 拉开距离、再倒车 7.3 m」，第一段结束时距预备
# 位姿本来就该从 5.50 m 涨到 6.00 m。所以：
#   · 至少要执行过 2 段才启用（第 1 段本来就允许变远）；
#   · 容差给到 2.0 m（约半个车身长），真正跑错方向才会超。
ALIGN_BEST_MARGIN = 2.0
ALIGN_BEST_MIN_SEG = 2
# 横向跑偏上限：超过就立刻重规划本段，而不是等这一段「走完」。
ALIGN_LAT_ABORT = 0.80
# [同学 C §6.36] ALIGN 的**出口门限**：横向残差超过它就不许「按当前位置直接入位」。
# 真机 B3（2026-10-29）的失败链：路线交付预备位姿时横向差 1.17 m → ALIGN 擦着
# `parked_b2` 走（侧面间隙 0.01 m、87 次护栏）→ 45 s 超时 → 旧代码无条件「按当前
# 位置直接入位」→ 带着 1 m 横向偏差起弧 → 249 s 内 1644 次护栏、连做 7 次重规划
# 才揉进车位，终点虽准（0.11 m）但最小车体间隙 0.00 m。B1 同工况横向 1.01 m，
# 只是 ALIGN 恰好没蹭上，说明 1 m 级的偏差本身就在赌运气。
# 0.60 m 的依据：车位深 2.4 m、车宽 1.85 m，单侧净空 0.275 m；横向残差 0.60 m
# 已经是净空的 2 倍以上，再带进去必然要靠「贴边揉库」找回，不可接受。
ALIGN_HANDOFF_LAT_MAX = 0.60
# 退回路线重接入的次数上限。为什么不是 0：一次重接入 = 重新走完最后十几米
# （含 S 形接入），成本约 20~30 s，比重规划一把揉库便宜两个数量级。
ALIGN_REAPPROACH_MAX = 2
# ALIGN 段内「贴合护栏」的**累计**次数门限（不是连续）：到点就判本段不可执行，
# 提前回上层，而不是耗到 45 s 超时。真机 B3 的 87 次分布在 45 s 里（约 2 次/s），
# 30 次≈15 s，既能滤掉「偶尔擦一下」的正常抖动，又能在真正刮擦时及早止损。
ALIGN_GUARD_ABORT = 30
# [同学 C §6.36] 路线末端「S 形接入」的幅度上限（m）。南侧道路的车道中心在
# y=-35.8，而平行位的预备位姿在 y=-34.70 —— 差 1.10 m，旧代码的补齐判据
# （法向 ≤0.6 m 才补直段）直接放弃，于是每个平行位的交接都自带 1.1 m 横向残差。
# 这里改成「末端 N 米内做平滑横移」，把残差在路线阶段就消掉。
STAGE_BLEND_LAT_MAX = 2.2      # 横向可修正幅度上限
STAGE_BLEND_ALONG_MAX = 4.0    # 纵向可修正幅度上限
STAGE_BLEND_LEN = 12.0         # 混合段长度（1.10 m 横移对应的最大曲率≈0.046，R≈22 m）
# 每一把（每个换挡段）至少要能走这么远，否则这条解**跟不动**：
# 换挡门限 `shift_gate_arc=0.70 m`，每段只有 0.7~0.8 m 时跟车器连「换挡完成」
# 都判不出来。实测 W4 退化出的「2.32 m / 2 换挡」（每段 0.77 m）就是这样：
# 原地反复换挡，最后 1.27 m / 25°、车体只剩 0.04 m。低于这个长度就换别的解。
MIN_SEG_LEN = 1.2

# 直连解 vs 两段式解「余量差多少才值得为它多揉一把」。
# 2.5 m 车位的净余量只有 0.325 m/侧，规划余量差 0.15 m 已经是两个量级的
# 执行风险（离线实测：W7 直连 +0.30 执行后只剩 +0.03，两段式 +0.55 剩 +0.22）。
SAFE_CLEAR_MARGIN = 0.15

# [同学 C 本轮 §6.29] 中心位姿 → 后轴位姿的纵向偏移（= 轴距/2）。
# HybridAStar 自本轮起以**后轴**为参考点（运动基元本来就是后轴积分，碰撞
# 轮廓已同步沿纵轴 +AXLE_OFFSET 平移；动机与推导见 hybrid_astar 模块头），
# 而定位、车位目标 goal_pose、预备位姿、路线图全是**中心**语义 —— 所有
# 进/出规划器的位姿都要过 `_rear()` 这道换算。控制器侧的对应物是
# `follower_park(ref_ahead=0)`（controller.update 入口把中心位姿换算到后轴）。
REAR_OFF = WHEELBASE / 2.0

# 入位判定：位置 ≤0.35 m、航向 ≤9° 才算真正停进车位（车位宽 2.5 m、平行位长 6 m，
# 0.35 m 的横向偏差仍在车位内）。以前「微调次数用尽就按当前位姿结束并把状态置成
# DONE」，实测出现过 0.42 m / 19.3° 也报 DONE 的情况 —— 画面里车明明是斜的。
# 现在不达标就报 INCOMPLETE，宁可显示失败也不能谎报成功。
DONE_XY_TOL = 0.35
DONE_YAW_TOL = math.radians(9.0)
# [同学 C 本轮] 入位全程「车体到障碍的最小距离」下限（m）。
# 终点误差与航向管的是「停得正不正」，管不了「是不是擦着邻车进去的」。
# 用户要求按 **0.2 m** 验收：整段入位过程中，车体矩形到最近障碍的最小距离
# 不得低于 0.20 m，否则这次泊车记 INCOMPLETE（哪怕终点误差很小）。
# 参考：2.5 m 车位停正时两侧各 0.325 m，所以 0.20 m 是「有明显余量」的门限。
MIN_PARK_GAP = 0.20
# [本轮修复 · 建议 2] 规划解「可执行余量」硬门限（m）。
# `plan_min_clear`（0.30）是「两个候选解之间怎么挑」的相对门限，只有两个解
# 都不达标时它就会退化成「矮子里拔将军」—— 真机 P1 连续三轮都拿到
# **车体余量 +0.05 m** 的解并照原样执行，结果终点偏 3.03 m、最小间隙 0.00 m。
# 这里再加一道**绝对**门限：低于它就不执行，先扩大搜索预算重搜（最多 3 次）。
MIN_EXEC_CLEAR = 0.30
# 平行位解析解的余量目标（m）。平行位两侧都是邻车，车位深 2.4 m、车宽 1.85 m，
# 单侧净空只有 0.275 m —— 比垂直位（2.5 m 车位、0.325 m/侧）更紧。
# [B3 修复调整] 0.45 → 0.50：位内横向间隙的几何上限是 2.4−1.85=0.55 m
# （两车都完美居中且平行），0.45 会接受「余量 0.46~0.50」的解，真机 B1 就是
# 拿着这种解执行后车体最小间隙只剩 0.21 m（阈值 0.20，擦线通过）—— 执行
# 侧的跟踪误差会吃掉 0.2 m 以上。门限抬到 0.50 把这部分误差预算要回来。
# **不能**抬到 0.70：那超过几何上限，所有解都会被拒收、重搜 3 次后放弃。
PARALLEL_CLEAR_TARGET = 0.50
# 路线段被激光护栏连续挡住多久就主动脱困（s）。真机 SW1 在 (-26.0, 9.9) 与
# 干扰车互相让行，自车末端 100 帧位移 0.000 m、白等 309 s 直到路线超时。
# [补丁] 20 s → 45 s：路线段正常让行（干扰车 / 行人）本来就会停十几秒，
# 20 s 会把正常让行也判成僵持，一停就提前交接，反而把车丢在半路上。
ROUTE_GUARD_ESCAPE = 45.0
ROUTE_GUARD_ESCAPE_MAX = 2   # 一次路线最多提前交接几次
# 提前交接只在「已经开到目标附近」时才允许（m）。否则自车在几十米外就被
# 丢给局部规划器，Hybrid A* 的 ±15 m 窗口根本覆盖不到车位；平行位还会
# 触发一次 60 m 的 ALIGN（真机 B3 a3 的失败原因）。
ROUTE_GUARD_ESCAPE_DIST = 18.0
# [B3 修复 · 路线段倒车脱困] 「原地被挡」累计多久就主动倒车退出（s）。
# 真机 B3（本轮）：自车沿西街行驶时跑偏到**西侧路缘**上（world −28.97, 1.09，
# 航向 −113.6°），车头正对着路缘，激光护栏把「前方 2.47 m / 横向 −0.87 m」
# 判成必撞 —— 这个判据本身没错（障碍就在车身横向范围内），但车一旦停住，
# 阿克曼底盘**静止时打不动方向**，于是直行必撞、原地永远是撞：护栏日志连续
# 刷了 372 s（累计 10061 次），交接/绕行两种脱困都改变不了「车头朝路缘」
# 这个几何事实，最后只能等路线超时。
# 唯一能破局的动作是**倒车**：退出来才有横移余量重新打方向。
ROUTE_BACK_WAIT = 20.0
# 一次倒车退多远（m）。2.5 m 足够把车头从「贴住路缘」退到「有转弯余量」。
ROUTE_BACK_DIST = 2.5
# 一次路线最多倒车几次。每次退完都重新规划路线，正常情况下 1 次就够；
# 给 4 次是防止在极窄处反复蹭，同时保证不会无限倒车。
ROUTE_BACK_MAX = 4
# 判定「真的往前走了」的门限（m，**沿路线剩余里程**的减少量）：被挡期间
# 只有路线剩余真的减少这么多才清零计时，否则激光偶尔一帧放行、车原地来回蹭
# 十几厘米，就会把「连续被挡」反复清零（真机 B1：护栏每 2 s 交替拦/放，
# 剩余恒为 18.7 m，计时永远攒不到 20 s，倒车脱困一次都没触发）。
ROUTE_BLOCK_RESET_DIST = 1.0
# 平行位「强制 ALIGN」的距离上限（m）。超过这个距离说明还没开到车位附近，
# 先做对齐机动没有意义（ALIGN 是最后几米的摆正动作）。
PARALLEL_ALIGN_MAX_DIST = 15.0
DOCK_RETRY_MAX = 5         # 车位内微调次数上限
# 两次「进度停滞 → 重规划」之间的最小间隔（s）。停滞是**状态**不是事件，
# 不节流的话微调预算会按控制周期（0.05 s/拍）消耗，几次就在一秒内烧光。
# 给 4 s：既够跟踪器重新起步、也够激光看清间隙有没有真的变化。
STUCK_RETRY_GAP = 4.0
END_RETRY_MAX = 4          # 「路径走完但没到位」补规划次数上限
RECOVER_MAX = 2            # 「退回预备位姿重新入位」次数上限
# [B3 修复] 同一车位连续规划失败几次就放弃本次泊车。强制指定车位时黑名单
# 不生效，没有这道闸就会「选位→失败→拉黑→又选中同一个位」无限循环。
PLAN_FAIL_MAX = 3

# 已经贴进「接触区」时允许继续执行的时间上限（秒）。见 `_side_guard_ok`：
# 停车判据用「当前位置到障碍的距离」，所以一旦真贴上，第一拍就会被判死；
# 给一个很短的放行窗口让车「往外挪/揉方向」出来，而不是原地锁死。
ESCAPE_SECONDS = 1.2

# [B3 修复 · 冻结等待] 入位段进度停滞时，若挡路的是外部干扰车（有倒退让行
# 逻辑的车，见 scene_agents/path_agent.py），原地冻结等待而不是烧微调预算。
# 真机 B3 时间线：自车入位弧中途被干扰车贴身封死（车体间隙 0.03 m），蓝车
# 倒退让路在 4093.98 s 完成，而自车 4096.03 s 已耗尽 5 次微调预算放弃 ——
# 只差 5 s。干扰车一轮「让行 15 s + 倒退 12 m@0.6 m/s + 停留 15 s」约 50 s，
# 上限取 75 s 覆盖一轮半；超时才转回常规停滞处理，不会永久冻结。
AGENT_FREEZE_MAX = 75.0
# 护栏命中点（车体系最近障碍点）与干扰车中心的匹配半径（m）。
# [B3 修复] 3.0 → 5.5：原先按「中心距」判，只覆盖到「对方车身中部蹭到自己」
# 的情形；而真机 B3 是**车头右前角 vs 对方车头左角**的斜向接触，两车中心
# 相距 4.3 m（各自半长 2.175），超出 3.0 m 就判不出「挡路的是会动的车」，
# 于是自车照旧把微调预算烧光（本轮 5 次、24 s，期间干扰车一次都没让开）。
# 5.5 = 两个半长之和 + 1.1 m 余量，覆盖对角接触。
AGENT_HIT_MATCH = 5.5
# 兜底：只要某台干扰车的**中心**离自车中心这么近，也认为「它在自己身边」。
# 斜向接触时命中点落在自车角上、对方车身其余部分却远在车的另一头，
# 单靠命中点匹配仍有漏网；而干扰车只有两台、又都会主动倒退让行，
# 这条兜底不会误伤静态邻车（静态车根本不在 self._agents 里）。
AGENT_NEAR_EGO = 7.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def geometry_point(x, y, z=0.0):
    p = Point()
    p.x, p.y, p.z = float(x), float(y), float(z)
    return p


def _dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def stage_offset(pose, stage):
    """自车位姿相对预备位姿的**纵向 / 横向**偏差（预备位姿自身朝向为基准）。

    为什么不能再用欧氏距离（§6.31 真机 W7）
    --------------------------------------
    入位弧的起点就是预备位姿，车差多少，跟踪器第一拍就有多大的初始横向误差。
    欧氏距离把「沿过道差 1.13 m」与「横向偏 1.13 m」当成同一件事，于是真机 W7
    在**航向完全对齐、只差 1.13 m 纵向**时仍被判「已在容差内」并锚定：跟踪器
    从 e=-0.69 m 起步，14.42 m 的 0 换挡倒车弧（规划余量只有 +0.30 m）侧向扫过
    邻车，执行间隙 5.31→0.28→0.01 m，判 INCOMPLETE。

    分解之后：
      · 横向（垂直过道）偏差才是真正吃掉邻车间隙的量，必须严格（≤0.30 m）；
      · 纵向偏差同样让车「不在弧的起点上」，但它是一个可以**直着开过去**补掉的
        量（ALIGN 一把直线就收敛），所以同样纳入判据，超差就交给 ALIGN。

    返回 `(along, lat)`：along>0 表示预备位姿在车头前方，lat>0 在车体左侧。
    """
    dx, dy = stage[0] - pose[0], stage[1] - pose[1]
    c, s = math.cos(stage[2]), math.sin(stage[2])
    return c * dx + s * dy, -s * dx + c * dy


def dedup_path(path, eps=1e-3):
    """去掉相邻的重复点（间距 < eps）。

    [同学 C 修复] Reeds-Shepp 的路径末尾常带一个与终点重合的点，两段式拼接
    时又会把「第一段终点 = 第二段起点」再重复一次，于是路径里出现**长度 0 的
    线段**。跟车器的垂足投影、剩余里程、换挡门限全部按弧长算，遇到 0 长度段
    会卡在同一个垂足上：`remaining` 不再减少 → 5 s 后判 `stuck`。
    离线复现（`verify_park_sim.py W7`）：车只走了 0.14 m 就停住，终点误差
    12.47 m、航向差 90.7°，正是「两段式拼接」那条分支。
    """
    out = []
    for p in path:
        if out and _dist(out[-1], p) < eps:
            continue
        out.append(tuple(p))
    return out


def min_leg(path):
    """路径里**最短的那个换挡段**的弧长（m）。

    用来识别「起步小段」：Reeds-Shepp 在起点位姿略微偏一点时会给出
    「先前进 0.1~0.4 m 再倒 12 m」这种解。这一小段对几何毫无帮助，却会让
    跟车器在换挡门限（0.70 m）上直接跳过去，`idx` 一跳、参考点就落到倒车弧
    里两三米处，`φ` 凭空变成十几度，曲率立刻贴限幅 —— 真机 W7 的
    「κ 连续 railed、横向偏差 3 m 内涨到 0.97 m、剐邻车」就是这条链路。
    """
    if len(path) < 2:
        return 0.0
    lens, g, acc = [], path[0][3], 0.0
    for a, b in zip(path, path[1:]):
        acc += _dist(a, b)
        if b[3] != g:
            lens.append(acc)
            g, acc = b[3], 0.0
    lens.append(acc)
    return min(lens)


# ---------------------------------------------------------------- 车体 / 护栏
HALF_L = 2.175          # 车长一半（4.35 m）
HALF_W = 0.925          # 车宽一半（1.85 m）
GUARD_M = 0.20          # 激光护栏余量
LIDAR_DX = -0.15        # 激光安装位置（车体系纵向偏移，见 ego_car/model.sdf）


def corridor_hit(ranges, angle_min, angle_inc, gear,
                 half_l=HALF_L, half_w=HALF_W, margin=GUARD_M, lidar_dx=LIDAR_DX,
                 ahead=None):
    """激光点是否落在「车体（外扩 margin）行驶方向那一半」里。

    纯函数，便于脱离 ROS 单测（见 `scripts/selftest_guard.py`）。
    `ahead` 可以放大纵向阈值：巡航段要提前几米停（等效跟车距离），
    入位段必须贴近车体才停（否则停不到位）。

    [同学 C 修复] 外扩余量**只在近处生效**（一个车身长以内）。
    旧写法对整条 4.5 m 走廊都用 `半宽 + 0.20 m`，于是「贴着车道边走」的东西
    —— 马路牙子、路缘石、扫过的邻车道停驻车 —— 只要在 4.5 m 内就会被当成挡路。
    实测一次运行里自车在离路缘只剩 **0.155 m** 的地方被这条判据冻死 **62 s**
    （新增的诊断日志把命中点报成 `lon=3.68 lat=+1.08`，一眼就能看出是平行路缘
    而不是前方障碍）。物理上 0.155 m 的侧向间隙车是能过去的：它不在行进路径上。

    现在分两层：
      · `|lat| < 半宽`（车身真要撞上）→ 整个走廊长度上都算挡；
      · `半宽 ≤ |lat| < 半宽 + 余量`（只是余量被吃掉）→ 只在 **一个车身长以内**
        算挡，远处的东西车会从旁边过去。
    """
    hl = (ahead if ahead is not None else half_l) + (0.0 if ahead is not None else margin)
    hw_soft = half_w + margin
    hl_soft = half_l + 0.25          # 余量生效的纵向范围：约一个车身长
    for i, r in enumerate(ranges):
        if not (0.06 < r < hl + 1.5):      # inf/nan 在这里自然被排除
            continue
        a = angle_min + angle_inc * i
        lon = r * math.cos(a) + lidar_dx
        if gear < 0:
            lon = -lon                     # 倒车看车尾那一半
        if not (-0.20 < lon < hl):
            continue
        lat = abs(r * math.sin(a))
        if lat < half_w or (lat < hw_soft and lon < hl_soft):
            return True
    return False


def body_clear(ranges, angle_min, angle_inc,
               half_l=HALF_L, half_w=HALF_W, lidar_dx=LIDAR_DX):
    """激光点到**车体矩形**的最小距离（m），并返回最近点的 (lon, lat, r)。

    [同学 C 修复] 「入位段总是剐蹭邻车」的直接原因：`_follow()` 里**根本没有**
    护栏调用 —— 激光只在巡航（`_route_tick`）和 ALIGN 里过一遍，真正的倒车入库
    是裸奔的。而 `corridor_hit` 不能直接搬过来：它要求「前方 `half_l + margin`
    米内全空」，可车位后端墙本来就只有 0.33 m，一搬过来车就永远停不到位。

    这里用**点到车体矩形的距离**（车体内为 0），而不是「横向 |lat| - 半宽」：
    后者对**正前/正后**的回波会给出一个毫无意义的负值（实测 `lat≈0` 时算出
    -0.92 m，看起来像「已经剐上」，其实那是正前方 2 m 处的墙）。矩形距离对
    侧向、纵向都成立，且车位后墙在 0.33 m 外 → 距离 0.33 > 阈值，不会误触发。

    纯函数，便于离线单测（见 `scripts/selftest_guard.py`）。
    """
    best, at = float('inf'), None
    for i, r in enumerate(ranges):
        if not (0.06 < r < 8.0):
            continue
        a = angle_min + angle_inc * i
        lon = r * math.cos(a) + lidar_dx
        lat = r * math.sin(a)
        d = math.hypot(max(0.0, abs(lon) - half_l), max(0.0, abs(lat) - half_w))
        if d < best:
            best, at = d, (lon, lat, r)
    return best, at


def sample_straight(p0, p1, step=0.1):
    """p0 → p1 的直线采样，方向字段按「沿车头为正」判定。

    [同学 C 修复] 旧实现把 direction 写死成 1（前进）。但这条直线段是从
    **预备位姿倒进车位**，位移方向与车头相反，必须是 -1：否则跟车器会用前进挡
    执行这一段，车头朝外却往车位里「开」，表现为原地兜圈/被 stuck 看门狗掐断。
    """
    dx, dy = p1[0] - p0[0], p1[1] - p0[1]
    d = _dist(p0, p1)
    d_sign = 1 if (dx * math.cos(p0[2]) + dy * math.sin(p0[2])) >= 0.0 else -1
    n = max(1, int(d / step))
    out = []
    for i in range(1, n + 1):
        t = i / n
        out.append((p0[0] + dx * t, p0[1] + dy * t, p1[2], d_sign, 0.0))
    return out


class ParkingPlanner(Node):
    def __init__(self):
        super().__init__('parking_planner')

        # ---------------- 参数 ----------------
        self.declare_parameter('layout_path', '')
        self.declare_parameter('world_path', '')
        self.declare_parameter('route_graph_path', '')
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)
        self.declare_parameter('pose_topic', '/vehicle_pose')
        self.declare_parameter('scan_topic', '/scan')
        # IMU：用来取真实偏航率给跟踪器做阻尼（见构造里订阅处的说明）
        self.declare_parameter('imu_topic', '/imu')
        # 在线搜索预备位姿时，「预备位姿到车位中心沿车位轴的距离」（m）。
        # 9.0 = 场内过道中心线到车位中心的距离，与车位图一致。
        self.declare_parameter('staging_along', 9.0)
        # 「机动锚定在预备位姿」允许的最大交接航向差（度）。
        #
        # [同学 C 本轮，有离线对照] 原来是 12°，实测定量拆开后改成 4°：
        # 规划弧的起始航向固定取预备位姿航向，自车真实航向差 Δψ 就成了**初始航向误差**。
        # 离线把 Δψ 从 −10° 扫到 +10°（`probe_yaw_sweep.py`，E3 与 W5）：
        #   E3：+0° → 横向峰值 0.14 / 间隙 +0.38；+6° → 0.45 / +0.12；+10° → 0.80 / **0.00**
        #   W5：+0° → 0.24 / +0.41；−6° → 0.62 / +0.20；−10° → 0.94 / **0.00**
        # 即「误差与转弯方向相反」的那一侧，每多 1° 就多漂约 0.08 m；
        # 而两侧完全镜像 —— 所以**不是跟踪律对反向弯不成立**，是曲率余量不够、
        # 误差帮着转时相安无事、顶着转时就切进邻车。
        # |Δψ| ≤ 3° 时两排都安全（间隙 ≥ +0.37）。
        # Δψ ≥ 6° 时改「从真实位姿规划」（`probe_anchor_yaw.py` 的 C 组）：
        #   E3 +8°：间隙 0.00 → **+0.41**；E8 +8°：0.00 → **+0.46**；
        #   W5 −8°：+0.03 → **+0.47**；W7 −8°：+0.03 → **+0.38**
        # 代价是多 1 次换挡，但余量从 0 变成 +0.4，完全值得。
        self.declare_parameter('anchor_yaw_tol', 4.0)
        self.declare_parameter('risk_topic', '/risk_level')
        self.declare_parameter('driver_status_topic', '/driver_status')
        self.declare_parameter('cmd_topic', '/cmd_vel_planner')
        self.declare_parameter('mode_topic', '/control_mode')
        self.declare_parameter('target_spot_topic', '/target_spot')
        self.declare_parameter('path_topic', '/parking_path')
        self.declare_parameter('status_topic', '/parking_status')
        self.declare_parameter('marker_topic', '/parking_markers')
        self.declare_parameter('free_grid_topic', '/parking_free_space')
        self.declare_parameter('goal_stall', '')
        self.declare_parameter('stall_region', '')       # lot / road_west / ...
        self.declare_parameter('prefer_stalls', '')
        self.declare_parameter('exclude_stalls', '')
        self.declare_parameter('wait_for_driver', False)
        self.declare_parameter('warmup_scans', 12)
        self.declare_parameter('max_speed', 0.45)
        self.declare_parameter('route_speed', 1.2)
        self.declare_parameter('control_rate', 20.0)
        self.declare_parameter('xy_res', 0.25)
        self.declare_parameter('th_res_deg', 3.0)
        self.declare_parameter('local_half_size', 15.0)
        # 规划用的车体余量（m）。0 = 膨胀半径恰好等于半个车宽，路径会贴到邻车
        # 3 cm 以内；0.15 m 是实测「垂直车位仍能停进去、且路径不再贴邻车」的取值。
        self.declare_parameter('plan_clearance', 0.15)
        # 选解用的**最低计划余量**（m）。见 `_plan` 里「余量优先」的说明：
        # 0 换挡不等于安全，只有余量够大才经得起跟踪误差。0 = 退回旧规则。
        self.declare_parameter('plan_min_clear', 0.30)
        # 入位规划用的**等效前轮转角**上限（rad）。0.44 → κ=0.1747、**R=5.74 m**。
        #
        # [同学 C 本轮，纯几何重算，不再沿用之前的结论]
        # 真机残余病根的形态是「车头还没转正就已经到了车位口，然后在口上继续转，
        # 车尾扫进邻车」。把它写成一个可算的量：**圆弧结束时后轴离车位口多远**
        # 以及**留给「直线摆正」的尾段多长**。机动 = 90° 圆弧(R) + 沿车位轴直线(9−R)：
        #   R=6.39 → 圆弧在车位口（x=2.61，口在 2.5）才转完，尾段只剩 2.61 m
        #   R=5.74 → 圆弧在 x=3.26 转完，尾段 3.26 m
        #   R=4.96 → 圆弧在 x=4.04 转完，尾段 4.04 m
        # 尾段是**零参考曲率**的一段 —— 车辆全部曲率预算（0.2019）都留给修偏差，
        # 所以尾段越长，「带着几度航向差进库」的容错越大。
        # 同时几何扫掠（`probe_sweep_envelope.py`，真实矩形对真实矩形）表明
        # R 越小、两侧邻车余量**只增不减**：
        #   R=4.96→+0.65  R=5.74→+0.65  R=6.39→+0.62  R=8.00→+0.54  R=9.00→+0.34
        # 也就是说「半径越大越安全」是错的，恰恰相反。
        # 取 0.42（R=6.05）：在车辆能力（κ 上限 0.2019）之内留 18% 反馈余量。
        # [同学 C §6.25] 0.44 → 0.42：0.44 的 κ=0.1747 占限幅 86%，E2 真机
        # 换挡瞬态 0.16 m 偏差就让指令顶死、横向偏差单调发散（101 m/INCOMPLETE，
        # 离线 HANDOFF_LAT=-0.2 完全复现）。0.40 的 R=6.39 超过过道半宽 6.25，
        # 两段式 ALIGN 机动物理上转不开（§6.19）；0.42 是「余量够 + 过道内
        # 仍可行」的平衡点（E2 需配合 staging_poses.yaml 的 -3 侧位姿）。
        self.declare_parameter('plan_max_steer', 0.42)
        # 巡航段行人让行距离（m，行人到**车体矩形**的距离）。0 = 关闭。
        self.declare_parameter('ped_yield', 2.5)
        # 入位段的行人让行距离：比巡航段小得多，只在「人已经贴到车身旁边」时停，
        # 否则行人路过车位口就会把揉库动作反复打断。
        self.declare_parameter('ped_yield_park', 1.5)
        # 行人让行扇形的**全角**（度）。<180° 才符合「只关心自己行进方向」的
        # 直觉；180° 等于把旧的圆形判据原样搬过来，>180° 会连车后的行人都拦。
        self.declare_parameter('ped_fov', 140.0)
        # 巡航段激光护栏的前视距离：等效「跟车距离」。取 4.5 m 是为了在行人/
        # 车辆前方提前停住 —— 只在 0.2 m 处拦的话，车会一直开到人脚底下才停，
        # 实测因此把自车卡在主干道上冻结 110 s（护栏连打 4300 次）。
        self.declare_parameter('guard_ahead', 4.5)
        self.declare_parameter('eval_csv', '')
        # 入位段的横向跑偏门限。**必须和计划余量（plan_clearance=0.15 m）同一
        # 量级**：原来 1.5 m 意味着车可以偏离计划线 1.5 m 还继续开 —— 而车位半宽
        # 只比车宽多 0.325 m，这 1.5 m 足够把邻车刮两遍（用户看到的「总会剐蹭」）。
        # 0.8 m 是折中：既能在真跑偏时及时重规划，又不会把换挡瞬间的假性冲高
        # （参考点前跳，瞬时 1.5 m 以上）误判成跑偏。
        self.declare_parameter('replan_tolerance', 0.8)
        # 泊车段「车体到最近障碍小于这个值就停车」的硬护栏（m）。0 = 关闭。
        # 取值史：0.10 → 0.03 → **0.12**。
        #   · 0.10 时，规划余量本身只有 0.15 m、跟踪残差 5~10 cm 很常见，于是
        #     「停滞 → 重规划 → 同一份解 → 再停滞」原地死循环（四轮真机全卡住）。
        #   · 0.03 能跑，但视觉上就是「贴着邻车 3 cm」，用户截图里看起来像蹭上了。
        #   · 现在取 0.12：`_side_guard_ok` 已有「不更近就放行」的脱困逻辑，
        #     不会再锁死；车位两侧净空 = (2.5−1.85)/2 = **0.325 m**，0.12 仍能
        #     停到位（留 20 cm 余量），同时不会再出现「贴着邻车」的画面。
        self.declare_parameter('park_side_gap', 0.12)
        # 是否在「路线 → 入位」的交接处先跑一次 ALIGN 对齐机动（默认**开启**，
        # 见 `_handoff` 的说明：先精确摆到预备位姿，入位才是 0 换挡的直线倒车）
        self.declare_parameter('use_align', True)
        # ALIGN 只在预备位姿已经在这个距离以内才跑（m）。
        # [同学 C] 试过放宽到 8.0（让场内垂直位 5.5 m / 80° 也走 ALIGN），
        # 真机实测**不收敛**：ALIGN 启动了，但每段只前进 1.1~1.3 m，重规划出来的
        # 总长一直停在 7.9~8.4 m，6 段 / 45 s 后超时放弃，再退回直接入位。
        # 根因是那套机动是「前进 1.1 m → 倒车 6 m → 再前进」，而 1.1 m < MIN_SEG_LEN，
        # `_trim_path` 每段都在那一小步之后切断。trim 已改成「太短的一把连同下一把
        # 一起包进来」，但这一版还没在真机上复验，所以先保守保持在 2.5 m：
        # 路侧平行位（1.6 m）走 ALIGN 一段收敛，场内垂直位直接入位。
        self.declare_parameter('align_max_dist', 2.5)
        self.declare_parameter('publish_free_grid', True)

        gp = self.get_parameter
        self.spawn = (float(gp('spawn_x').value), float(gp('spawn_y').value),
                      float(gp('spawn_yaw').value))

        world_layout = self._load_layout(gp('layout_path').value, gp('world_path').value)
        self.world_layout = world_layout
        self.layout = world_layout.to_frame(self.spawn)
        self.get_logger().info(
            f'布局已变换到 map 帧：spawn={tuple(round(v, 2) for v in self.spawn)}，'
            f'车位 {len(self.layout.stalls)} 个'
            f'（场内垂直 {len(self.layout.stall_ids("lot"))}、'
            f'西街 {len(self.layout.stall_ids("road_west"))}、'
            f'东街 {len(self.layout.stall_ids("road_east"))}、'
            f'南街 {len(self.layout.stall_ids("road_south"))}）')

        self.detector = SpotDetector(self.layout)
        # 注意 max_steer 用 FOLLOW_MAX_STEER（0.50），不是规划用的
        # EFFECTIVE_MAX_STEER（0.44）：跟踪器的限幅必须比规划曲率大一档，
        # 否则在最大曲率段上增益把余量吃干，闭环永久饱和。
        self.follower_route = PathFollower(
            wheelbase=WHEELBASE, max_steer=FOLLOW_MAX_STEER,
            v_max=float(gp('route_speed').value), v_min=0.3,
            # 路线跟踪用更长的前视 + 更大的反馈增益：场内过道的 90° 转角受可用
            # 直线长度限制，实际生成的圆弧半径只有 4 到 6 m，接近车辆能力极限，
            # 前视太短会在出弯时留下 3 m 的横向偏差，导致后面 Hybrid A* 要在一个
            # 很偏的起点上做十几米的揉库规划（又慢又容易失败）。
            lookahead_min=2.2, lookahead_gain=0.8,
            # [同学 C 修复] 终点判据 0.8 m / 25° → 0.20 m / 6°。
            # 松弛的终点判据是「交接位姿总差 0.4 m / 12°」的直接原因：路线跟车器
            # 只要「剩余 <0.2 m 且距终点 <0.8 m、航向差 <25°」就宣布 done，
            # 于是车还在预备位姿斜后方 0.4 m、歪 12° 时就交接了。
            # 而垂直车位的净余量只有 0.325 m/侧（车位 2.5 m、车宽 1.85 m），
            # 交接差 0.1 m 就直接从这点余量里扣；离线用真实交接位姿扫过
            # （`HANDOFF_BACK`）：W7 在 0/0.1/0.2/0.3 m 时执行间隙 +0.17/+0.03，
            # 到 0.4 m 就掉成 **0.00 m（剐邻车）**，且规划从 0 换挡退化成 1 换挡
            # （先往前挪一把）—— 真机 W7 卡死就是这条链路。
            goal_xy_tol=0.20, goal_yaw_tol=math.radians(6.0),
            # 增益保持原值 1.0 / 1.8：实测把它降到 0.45/1.2 后，路线跟踪横向
            # 偏差反而涨到 2.5~3.5 m（触发反复重规划、交接到 25 m 之外），
            # 所以「左右摇摆」不是增益太大造成的，降增益是有害的。
            # 真正的解法是 controller 里的两项：
            #   · k_rate 偏航率阻尼（微分项）—— 压住那个振幅会增长的振荡；
            #   · steer_rate 转向速率限制 —— 禁止方向盘在死左/死右之间瞬跳。
            # 上面两项只把摆动压小，没有断根。根因是**零预瞄**：参考点只有垂足，
            # 于是「横向偏差」和「航向偏差」同相位地推舵，而 map→base 的定位残差
            # （实测峰峰值 1.41 m、60 s 内符号翻转 8 次）正好喂进这个通道，
            # 舵就跟着残差一起摆。改成预瞄纯跟踪（preview=2.8 m）后，等效航向
            # 增益从 k_head=1.8 降到 2/L_p≈0.71，而且瞄准的是**前方路径点**，
            # 等于对定位噪声做了一次空间平均 —— 不会一偏就顶到曲率限幅。
            # 离线闭环（真实 PathFollower + 一阶转向滞后 + 20 Hz 采样 + 定位慢漂）
            # 实测：真值横向偏差 0.74→0.33 m，ω 符号翻转 21→8 次（60 s）。
            # smooth=0.20 s：舵指令一阶低通，再压掉一半的翻号，幅度不受影响。
            # 入位段（follower_park）坚决不开这两项：实测泊车终点误差会从
            # 0.32 m 恶化到 1.76 m，精度和抗摆是两套参数。
            k_lat=1.0, k_head=1.8, k_rate=1.2,
            preview=2.8, smooth=0.20,
            # 终点锚定修复后（_lookahead 直接返回路径末点，不再有浮点边界），
            # 末段 Ld 塌缩的增益由这个下限封顶：1.0 与预瞄收缩的下限一致。
            ld_floor=1.0)
        self.follower_park = PathFollower(
            # [同学 C 本轮] 入位段：舵角上限 0.50（= 车能给的最大曲率 0.2019）、
            # `steer_gain` 1.04（补台架量到的 4% 欠转）。推导见模块头部常量注释。
            # 这两个值必须与 `_plan` 的「机动锚定在预备位姿」一起用：锚定保证几何
            # 是那条验证过的 0 换挡圆弧，增益保证车真能转出这个曲率。
            wheelbase=WHEELBASE, max_steer=FOLLOW_MAX_STEER_PARK,
            steer_gain=PARK_STEER_GAIN,
            # [同学 C] 入位限速保持 0.45 m/s。
            # 本轮试过降到 0.35（理由是「航向环增益 ∝ 速度」），但真机实测 W4 连跑
            # 三次都拖过了超时窗口：降速只把整段机动拉长，并没有压住
            # 「8° 初始航向差把 κ 顶到限幅」这条主因（见 §6.19 的真机日志）。
            # 等交接航向差压到 ≤3° 之后再回头试降速才有意义。
            v_max=float(gp('max_speed').value),
            # [同学 C 本轮] 重新打开偏航率阻尼。之前关掉是在**没有真实偏航率信号**
            # 的前提下关的（位姿差分恒 0，k_rate 等于没开，却白白把闭环增益抬高了）。
            # 现在 `_yaw_rate()` 走 IMU（见那里），阻尼才真正起作用：
            # 它把「实测 ω」和「指令 ω」的差反馈回曲率，等于给那个 ±20° 的航向
            # 极限环加阻尼。前进/倒车都生效（controller 里已放开 gear 限制）。
            k_rate=0.8, steer_rate=0.0,
            # [同学 C 本轮] 入位段改用**预瞄纯跟踪**（和巡航段同一套律，preview=2.4 m）。
            # 道理和 controller 里那段注释一致：旧的 e/φ 反馈律等效航向增益
            # ≈ k_head = 1.8，是纯跟踪 2/L_p≈0.83 的两倍多；而这条圆弧的规划曲率
            # 0.1747 已经占掉车辆上限（0.2019）的 86%，反馈只剩 0.027 的余量 ——
            # 任何超过 2° 的航向差都会把 κ 顶到限幅，于是「切进去 → 反向过冲」的
            # bang-bang 极限环。真机 W5（2026-10-27 15:57）就是把 κ 钉在 +0.202、
            # 横向偏差 4 m 内从 0.10 涨到 1.01 m、最后停在离车位 5 m 处擦到邻车。
            # 换纯跟踪后离线末端航向从 0.5° 收到 0.1~0.6°，W 排余量也从
            # +0.39 提到 +0.63（E3/E8 仍是规划几何的问题，见 §6.20）。
            preview=2.4,
            # 入位段不做末段收缩（tail_anchor=False），但预瞄够不到时仍**锚定
            # 在路径终点**（_lookahead 修复后不再死于浮点边界）；锚定后 Ld 随车
            # 逼近塌缩，2/L_d 增益由 ld_floor=1.2 封顶（增益 ≤1.67），定位横向
            # 残差不再被放大成顶满的 κ。要跨换挡点时仍退回 e/φ 反馈律。
            tail_anchor=False, ld_floor=1.2,
            # 到点判据 0.15 m / 6°：原来的 0.35 m / 10° 会让车提前 0.3 m 收油，
            # 终点误差直接被这个门限吃掉（离线：0.35/10° 时 0.25~0.34 m，
            # 0.15/6° 时 0.14~0.23 m）。这是「停到位」的最后一段。
            goal_xy_tol=0.15, goal_yaw_tol=math.radians(6.0),
            # [同学 C 本轮 §6.29] 入位段改**后轴参考**：规划器（hybrid_astar）
            # 自本轮起输出后轴轨迹，ref_ahead=0 让 controller.update 把反馈
            # 位姿换算到后轴 —— 后轴速度恒沿车体纵轴，β̂≡0，弧上不再有
            # 「航向压正、速度仍偏 12°」的发散通道，C|C 变号点也没有 2β 的
            # 速度方向瞬跳（E2 真机 R 段漂移的根因，见 hybrid_astar 模块头）。
            # 路线段（follower_route）保持默认中心参考（L/2，β 补偿开启）。
            ref_ahead=0.0)
        self.follower = self.follower_route

        gpath = gp('route_graph_path').value
        if not gpath:
            try:
                gpath = default_graph_path(get_package_share_directory('parking_planner'))
            except Exception:
                gpath = default_graph_path()
        self.route_graph = RouteGraph.load(gpath).to_frame(self.spawn)
        self.get_logger().info(f'车道图：{gpath}（{len(self.route_graph.nodes)} 节点，已变换到 map 帧）')

        self.planner = None
        self.grid = None

        # ---------------- 状态 ----------------
        self.state = 'WAIT_DATA'
        self.pose = None
        self.pose_stamp = 0.0
        self.risk = 'safe'
        self.risk_stamp = 0.0
        self.driver_state = ''
        self.driver_stamp = 0.0
        self.scan_count = 0
        # 最近一帧激光（ranges, angle_min, angle_increment），供车体护栏使用
        self.scan = None
        self._guard_log = 0.0
        self._guard_hits = 0
        # 本次「连续被护栏拦住」的起始时刻（None = 当前没被拦），用于诊断僵持
        self._guard_since = None
        # 路线段「护栏僵持脱困」已用次数（见 `_route_tick`，建议 5b）
        self._guard_escape_n = 0
        # 入位段侧向剐蹭护栏的状态（见 _side_guard_ok / lateral_gap）
        self._side_gap = float('inf')
        self._side_at = None            # 最近障碍的碰点 (lon, lat, r)
        # 整段入位过程中「车体到障碍的最小距离」，验收用（阈值 MIN_PARK_GAP）
        self.min_park_gap = float('inf')
        # 当前这条入位路径**规划器承诺**的最小车体余量（倒车护栏的相对门限）
        self._plan_clear = 0.50
        # 规划解法离谱时的重试计数（见 _plan）
        self._plan_retry = 0
        self._plan_wait_until = None
        self._side_hits = 0
        self._side_log = 0.0
        # 脱困放行（见 _side_guard_ok 的「正在变远就放行」）
        self._side_t = 0.0
        self._escape_ref = float('inf')
        self._escape_left = 0.0
        self._escape_hits = 0
        # 行人位姿（/pedestrians，PoseArray）。巡航段用它主动让行——
        # 激光护栏只能拦住「已经贴到车体」的情况，看起来仍是「车顶着人开」。
        self.peds = []
        self._ped_log = 0.0
        self.target = None
        self.path = []
        self.route = []
        self.plan_info = {}
        self.blacklist = set(x for x in gp('exclude_stalls').value.split(',') if x)
        self.prefer = [x for x in gp('prefer_stalls').value.split(',') if x]
        self.danger_since = None
        # 路线阶段「累计被遮挡时长」（s）：超过门限就切低速蠕行，见 _route_tick
        self._danger_total = 0.0
        self.start_time = time.time()
        self.finish_time = None
        self.retry_at = 0.0
        self._evals = None
        self._evals_stamp = 0.0
        self._last_dbg = 0.0
        self._last_cmd_log = 0.0
        self._lat_bad = 0
        self._route_lat_bad = 0.0
        self._route_len = 0.0
        self._replan_left = 2
        # 到点对齐的次数与上一次时间戳（见 DOCK_AT_STAGE_* 常量说明）
        self._aim_left = DOCK_AT_STAGE_RETRY
        self._aim_time = -1e9
        self._align_len = 0.0
        self._align_retry = 0
        self._align_n = 0
        self._align_anchor = (0.0, 0.0)
        self._align_best = float('inf')     # ALIGN 里距预备位姿的历史最好成绩
        self._align_prev_d = float('inf')   # 上一段结束时的距离（判「平台期」）
        self._align_stall = 0               # 连续「没有改进」的段数
        # [§6.36] ALIGN 的出口控制：横向残差超限就退回路线重接入（不是带偏入位），
        # 以及「本段累计贴了几次护栏就判不可执行」
        self._reapproach_left = ALIGN_REAPPROACH_MAX
        self._align_guard_n = 0
        # 位姿瞬断（stale）被强制停住的累计时长：不计入路线行驶超时
        self._pause_since = None
        self._pause_total = 0.0
        # [同学 C 本轮] 为行人/障碍让行的累计时长，同样不计入路线行驶超时。
        # 行人 5 组、每组在进场车道能站十几秒，一轮很容易累计 100 s+；
        # 路线超时按长度只有 90+79.5/0.6 = 222 s，实测就被这段「该等的时间」判死。
        self._yield_total = 0.0
        self._route_remaining = 1e9      # 上一拍的剩余里程（末段减速用）
        self._escape_pending = False
        self._escape_left = 3
        self._no_progress = 0.0
        self._reenter = False
        self._recover_left = RECOVER_MAX
        self._dock_retry = 0
        # 「路径索引走完、但车还没到终点」的独立重试计数。
        # 不能和 _dock_retry 共用一个：后者在一次入位里会被换挡/停滞用掉，
        # 等到最后真正需要补那半米时已经用尽，只能 INCOMPLETE 收场。
        self._end_retry = 0
        self._stuck_hits = 0
        # [B3 修复] 入位段「冻结等待外部干扰车让路」的起始时刻（None=不在冻结中）
        self._freeze_t0 = None
        self._freeze_log = 0.0
        # [B3 修复] 路线段「原地被挡」累计计时与倒车脱困状态
        self._route_blk_anchor = None    # 计时锚点：净位移超过门限才换锚
        self._route_blk_t = 0.0
        self._route_back_from = None     # 非空 = 正在倒车脱困（记录起点）
        self._route_back_t0 = None       # 本次倒车脱困的起始时刻（时长上限）
        self._route_back_n = 0
        # [B3 修复] 上次「进度停滞 → 重规划」的时刻（按时间节流，见 _follow）
        self._stuck_t = 0.0
        # [B3 修复] 同一车位连续规划失败计数（防强制车位时无限重选）
        self._plan_fail_n = 0
        self._prev_ctrl = None
        self._yaw_rate_meas = None      # 实测偏航率（诊断/标定用）
        self.imu_wz = None              # IMU 车体偏航率（rad/s，+ 为逆时针）
        self.imu_stamp = 0.0
        self.imu_miss = None            # 首次发现「没 IMU」的时刻（只警告一次）
        self._imu_logged = False
        self._follow_start = time.time()
        self.recon = False

        # ---------------- 通信 ----------------
        self.create_subscription(PoseStamped, gp('pose_topic').value, self.on_pose, 20)
        self.create_subscription(LaserScan, gp('scan_topic').value, self.on_scan, 10)
        self.create_subscription(String, gp('risk_topic').value, self.on_risk, 10)
        # [同学 C 本轮] IMU：拿**真实偏航率**。
        # `/vehicle_pose` 是 `map→odom`（slam_toolbox，只在扫描匹配出校正时才变）
        # 与 `odom→base`（阿克曼插件，两侧后轮同速 ⇒ 偏航率恒 0）复合出来的，
        # 实测 `_yaw_rate` 逐帧差分出来**一直是 0.00**。于是
        #   · 跟踪器的偏航率阻尼 `k_rate` 只能关掉（喂 0 等于没有阻尼）；
        #   · 航向环变成一个**纯延迟环**（延迟＝一次 SLAM 校正，按 0.45 m/s 约 1 s），
        #     增益 1.8×0.45×1.0 ≈ 0.8，再加上 0.25 s 转向滞后必然过冲 ——
        #     真机 φ 在 −12°~+22° 之间来回摆就是这个环失稳。
        # IMU 是 100 Hz 的车体角速度，没有这个延迟，正好用来做阻尼项。
        self.create_subscription(Imu, gp('imu_topic').value, self.on_imu, 50)
        # 行人位姿：巡航段主动让行（见 _ped_in_front）
        self.create_subscription(PoseArray, '/pedestrians', self.on_pedestrians, 10)
        # [B3 修复] 两台干扰车的世界坐标位姿（gz_bridge 已桥接）。规划层不控制
        # 它们，只读位姿做让行决策：入位段判「进度停滞是不是它们挡路」
        # （见 _agent_hold），把「等别人让路」和「自己几何卡死」区分开。
        self._agents = {}
        for _topic in ('/traffic/odom_gt', '/traffic2/odom_gt'):
            self._agents[_topic] = None
            self.create_subscription(
                Odometry, _topic,
                lambda m, t=_topic: self._on_agent(m, t), 20)
        self.create_subscription(String, gp('driver_status_topic').value,
                                 self.on_driver_status, 10)

        self.pub_cmd = self.create_publisher(Twist, gp('cmd_topic').value, 10)
        self.pub_mode = self.create_publisher(String, gp('mode_topic').value, 10)
        self.pub_target = self.create_publisher(PointStamped, gp('target_spot_topic').value, 10)
        self.pub_path = self.create_publisher(Path, gp('path_topic').value, 10)
        self.pub_status = self.create_publisher(String, gp('status_topic').value, 10)
        self.pub_marker = self.create_publisher(MarkerArray, gp('marker_topic').value, 10)
        self.pub_free = self.create_publisher(OccupancyGrid, gp('free_grid_topic').value, 1)

        rate = float(gp('control_rate').value)
        self.create_timer(1.0 / rate, self.on_timer)
        self.create_timer(1.0, self.publish_viz)

        self.eval_csv = gp('eval_csv').value or os.path.expanduser('~/parking_eval.csv')

    # ------------------------------------------------------------------ 布局/栅格
    def _load_layout(self, layout_path, world_path):
        if layout_path and os.path.exists(layout_path):
            self.get_logger().info(f'读取先验布局：{layout_path}')
            return Layout.load(layout_path)
        try:
            share = get_package_share_directory('parking_planner')
            p = os.path.join(share, 'config', 'lot_layout.yaml')
            if os.path.exists(p):
                self.get_logger().info(f'读取先验布局：{p}')
                return Layout.load(p)
        except Exception:
            pass
        if world_path and os.path.exists(world_path):
            self.get_logger().info(f'布局文件缺失，直接从 world 解析：{world_path}')
            return Layout.from_world(world_path, 0.10)
        raise RuntimeError('既没有 layout_path 也没有可用的 world_path')

    def _new_planner(self, clearance=0.0):
        """在当前局部栅格上新建一个规划器。

        `clearance` 是车体余量：0 表示碰撞膨胀半径恰好等于半个车宽（最快，
        但路径可能贴着邻车只剩几厘米）；大于 0 时路径会主动离障碍远一点，
        代价是搜索变慢、紧车位可能要多揉一把。

        转角上限取参数 `plan_max_steer`（默认 0.42，见 declare 处的推导）。
        [同学 C §6.25] 历史教训：0.44 的 κ=0.1747 占执行限幅 86%，满曲率
        弧段上反馈余量只有 0.019 —— E2 真机换挡瞬态 0.16 m 偏差就让指令
        顶死、横向偏差单调发散。0.42 留 18% 余量，双向交接偏差都能收敛。
        """
        return HybridAStar(
            self.grid, wheelbase=WHEELBASE,
            max_steer=float(self.get_parameter('plan_max_steer').value),
            xy_res=float(self.get_parameter('xy_res').value),
            th_res=math.radians(float(self.get_parameter('th_res_deg').value)),
            # 平行车位的揉库解实测最多 780 次扩展就出来了，6000 足够；
            # 上限压小可以把「求解不了」的情况快速判死，而不是卡几分钟。
            max_iter=6000,
            clearance=float(clearance),
            max_cusps=3 if self.target.kind == 'parallel' else 2,
            # [同学 C §6.26] 垂直位的直连解析解按**真实车体余量**挑：最短解
            # 在 W5/W7/E3/E8 只有 +0.068 m（真机判据要 0.2 m），同起终点的
            # 其它 RS 模式族能给 +0.319 m。
            # [本轮修复 · 建议 3] 平行位**不再**给 0.0。原来注释写「余量目标不
            # 适用」，于是解析解只按最短挑 —— 真机 P1/P3 拿到的就是
            # 计划余量 +0.05 / +0.18 m 的解，执行后车体最小间隙 **0.00 m**。
            # 平行位两侧都是邻车（车位深 2.4 m、车宽 1.85 m，单侧只有 0.275 m），
            # 余量比垂直位更金贵，所以给一个比垂直位更大的目标。
            clear_target=0.30 if self.target.kind != 'parallel'
            else PARALLEL_CLEAR_TARGET)

    def _build_local_planner(self):
        """围绕「自车 + 目标车位」建局部栅格，避免整街区栅格化。

        栅格分辨率取 `xy_res` 的一半：碰撞检查的步长是 `xy_res`，但启发式距离场
        在更细的栅格上更平滑，实测搜索扩展数少很多（0.25 m 时同一个平行车位要
        多搜几十倍）。窗口压到 ±15 m 已经把距离场的规模控制住了。
        """
        half = float(self.get_parameter('local_half_size').value)
        pts = [self.pose, self.target.goal_pose, self.target.staging_pose]
        cx = sum(p[0] for p in pts) / len(pts)
        cy = sum(p[1] for p in pts) / len(pts)
        need = max(_dist(p, (cx, cy)) for p in pts) + 6.0
        half = max(half, need)
        roi = (cx - half, cx + half, cy - half, cy + half)
        self.grid = self.layout.occupancy_grid(
            float(self.get_parameter('xy_res').value) / 2.0, roi=roi)
        # [同学 C] 交接点诊断：把「自车 / 目标 / 预备位姿 / 窗口中心」四个坐标
        # 打出来。入位失败（起点被占据、2D 不连通）几乎都和这三者的相对位置
        # 有关，只看报错文字没法判断是谁跑偏了。
        gp_, sp_ = self.target.goal_pose, self.target.staging_pose
        self.get_logger().info(
            f'[局部窗口] 自车=({self.pose[0]:.2f},{self.pose[1]:.2f},'
            f'{math.degrees(self.pose[2]):.0f}°) '
            f'目标=({gp_[0]:.2f},{gp_[1]:.2f},{math.degrees(gp_[2]):.0f}°) '
            f'预备=({sp_[0]:.2f},{sp_[1]:.2f},{math.degrees(sp_[2]):.0f}°) '
            f'窗口中心=({cx:.2f},{cy:.2f}) 半宽={half:.1f} m '
            f'|自车-预备|={_dist(self.pose, sp_):.2f} m '
            f'|自车-目标|={_dist(self.pose, gp_):.2f} m')
        # 车体余量：垂直车位**必须**留，因为两侧只剩 0.325 m（车位 2.5、车宽
        # 1.85），不留余量时规划出来的路径实测只剩 +0.083 m（W9），跟踪误差
        # 一动就蹭邻车。平行车位的揉库本来就要挤，留余量会让搜索爆炸（实测
        # 加上 0.12 m 后平行车位规划从秒级退化成分钟级），那边维持 0。
        clr = (float(self.get_parameter('plan_clearance').value)
               if self.target.kind != 'parallel' else 0.0)
        # 选解门限：只有垂直车位才有「两侧各 0.325 m」这种余量问题
        # [本轮修复 · 建议 2/3] 平行位**不再是 0**。
        # 原来平行位 `plan_min_clear = 0.0`，等于「任何余量都算够」——
        # 12.8 m / 车体余量 +0.05 m 的揉库解因此被当成正常解执行（真机 P1
        # 终点偏 3.03 m、最小间隙 0.00 m）。但**栅格搜索余量**（上面的 `clr`）
        # 对平行位仍保持 0：模块注释里实测过「加 0.12 m 后平行车位规划从
        # 秒级退化到分钟级」，所以只把「选解/拒收」这道门限补上。
        if self.target.kind == 'parallel':
            self.plan_min_clear = PARALLEL_CLEAR_TARGET
        else:
            self.plan_min_clear = float(
                self.get_parameter('plan_min_clear').value)
        # 搜索预算的基准值：拒收后翻倍重搜，成功后要能恢复（见 `_plan`）
        self._base_max_iter = 6000
        self.planner = self._new_planner(clr)
        self.get_logger().info(
            f'局部规划窗口 {2 * half:.0f} m，栅格 {self.grid.width}x{self.grid.height}')

    # ------------------------------------------------------------------ 回调
    def on_pose(self, msg):
        p = msg.pose.position
        self.pose = (p.x, p.y, yaw_of(msg.pose.orientation))
        self.pose_stamp = time.time()

    def on_imu(self, msg):
        """车体偏航率（rad/s）。只用 z 轴角速度，不做姿态解算 —— 车在平地上跑。"""
        self.imu_wz = float(msg.angular_velocity.z)
        self.imu_stamp = time.time()
        if not self._imu_logged:
            self._imu_logged = True
            self.get_logger().info(
                f'IMU 偏航率已接入（{self.get_parameter("imu_topic").value}），'
                f'跟踪器的偏航率阻尼从此有真实信号')

    def on_scan(self, msg):
        if self.pose is None:
            return
        dt = getattr(msg, 'scan_time', 0.0) or 0.1
        self.detector.add_scan(msg.ranges, msg.angle_min, msg.angle_increment,
                               self.pose, dt=max(0.05, min(dt, 0.5)))
        self.scan = (msg.ranges, msg.angle_min, msg.angle_increment)
        self.scan_count += 1

    def on_pedestrians(self, msg):
        """行人位姿 -> map 帧。

        [同学 C 修复] `/pedestrians` 里发的是**世界坐标**（行人的闭环航点本身就是
        gz 世界坐标，直接 set_pose 过去的），而规划层内部统一用 map 帧。之前直接把
        世界坐标当 map 坐标存进 `self.peds`，等于给每个行人加了 (spawn_x, spawn_y)
        约 47 m 的偏移 —— 结果是「巡航让行」一次都没触发过（日志里恒为 0）：
        自车只靠激光护栏在 4.5 m 处被动停车，判别不出挡路的是人还是车。
        """
        sx, sy, syaw = self.spawn
        c, s = math.cos(-syaw), math.sin(-syaw)
        out = []
        for p in msg.poses:
            dx, dy = p.position.x - sx, p.position.y - sy
            out.append((c * dx - s * dy, s * dx + c * dy))
        self.peds = out

    def _ped_in_front(self, near=None):
        """让行判据：只对**行驶方向前方扇形**内、且离车体矩形小于 near 的行人停车。

        [同学 C 修复] 原来是无方向的：只要行人在车体矩形 near 米以内就停，跟在
        车侧 0.4 m 还是已经走到车后面无关。后果是「行人明明可以走开，车也在等」，
        两边互相等 —— 用户看到的「车辆需要调整位姿向外移」就是这么来的。
        真实驾驶里只关心**自己行进路径上**有没有人，视野不到 180°，这里取
        ±ped_fov/2（默认合计 140°）。倒车时行进方向是车尾，扇形整体转 180°。

        车体矩形对 180° 旋转是对称的，所以直接在「行进坐标系」里算矩形距离是
        精确的；只有落在车身内/贴住的行人（|lon|≤半长、|lat|≤半宽）无条件让行，
        安全兜底不参与扇形判断。
        """
        near = float(near if near is not None
                     else self.get_parameter('ped_yield').value)
        if near <= 0.0 or not self.peds or self.pose is None:
            return False
        gear = getattr(self.follower, 'gear', 1)
        th = self.pose[2] + (math.pi if gear < 0 else 0.0)
        c, s = math.cos(th), math.sin(th)
        half = math.radians(float(self.get_parameter('ped_fov').value)) * 0.5
        for (px, py) in self.peds:
            dx, dy = px - self.pose[0], py - self.pose[1]
            lon = c * dx + s * dy
            lat = -s * dx + c * dy
            if abs(lon) <= HALF_L + 0.15 and abs(lat) <= HALF_W + 0.15:
                return True                       # 已经贴住/压住车身：无条件让
            d = math.hypot(max(0.0, abs(lon) - HALF_L),
                           max(0.0, abs(lat) - HALF_W))
            if d >= near:
                continue
            if abs(math.atan2(lat, lon)) <= half:
                return True                       # 在行进方向扇形内
        return False

    def _lidar_guard(self, gear, ahead=None):
        """激光硬护栏：车体（外扩 GUARD_M）里只要扫到任何东西，就不给速度。

        为什么必须自己判：`self.risk` 只来自 `/risk`（detection→prediction→
        risk 那条链），而跑演示时默认 `use_prediction:=false`，那条链根本没起，
        `risk` 永远是 'safe'。于是自车会直接开到一个站着不动的行人身上
        （真机截图里行人就嵌在车顶）。安全不能依赖可选模块，这里用激光兜底：
        只做「不许碰到东西」这一件事，判据与规划器的车体模型一致。

        注意余量取 0.20 m 而不是更大：车位内倒车时，车尾到后方的墙/邻车本来
        就只有约 0.33 m，余量再大就会把「停到位」这一步也拦掉。
        """
        if self.scan is None or self.pose is None:
            return False
        ranges, amin, inc = self.scan
        return corridor_hit(ranges, amin, inc, gear, ahead=ahead)

    def _side_guard_ok(self, now, gear=1, remaining=0.0):
        """泊车段的车体贴合护栏（判据见模块级 `body_clear`）。

        两档：
          · `park_side_gap`（默认 0.12 m）：要贴上了才停；
          · **倒车「不该这么早贴近」档**（0.42 m）：只在 `gear<0`、泊车路径还剩
            2 m 以上、而障碍物就在**车尾正后方**时生效。理由见下面的注释 ——
            这是「倒车顶到干扰车还继续给油」的直接防线。
        """
        if self.scan is None or self.pose is None:
            return True
        ranges, amin, inc = self.scan
        gap, at = body_clear(ranges, amin, inc)
        dt = max(0.0, min(0.20, now - self._side_t))
        self._side_t = now
        gap_prev = self._side_gap
        self._side_gap = gap
        self._side_at = at
        # 验收用：整段入位的最小车体间隙（见 MIN_PARK_GAP）
        # [B3 修复] **动态代理不计入验收**。用户要 0.20 m 的判据针对的是
        # 「两侧有车的侧方位」里那两台停放车；而这段时间里干扰车/行人会从
        # 车旁经过，激光打到它们只有 0.05~0.15 m，于是「停得极准」的一轮
        # 也会被判 INCOMPLETE —— 真机 B5 a2：终点 0.11 m / 2.5°（判据
        # 0.35 m / 9° 全达标），却因为整段最小间隙 0.10 m 被记成未达标，
        # 而它那一轮的两侧邻车其实离得很远。安全护栏照旧对它们生效
        # （该停还是停），只是不拿它们去评判「泊车质量」。
        if not self._hit_dynamic(at):
            self.min_park_gap = min(self.min_park_gap, gap)
        limit = float(self.get_parameter('park_side_gap').value)
        if limit <= 0.0:
            return True
        # [同学 C 本轮新增] 倒车「不该这么早贴近」档。
        # 场景：E6 真机里自车**知道后方有黄色干扰车**（激光看得到），还是继续倒，
        # 顶上去之后还往前推。原因是泊车段只有 0.12 m 一档护栏 —— 而 0.12 m 是
        # 「已经到了」的门限，不是「该不该走」的门限。规划路径还差 2 m 的时候
        # 车尾就已经贴到 0.2 m，说明执行轨迹严重偏离了规划（规划保证 ≥0.30 m），
        # 这时候正确动作是停车重规划，而不是把最后 0.12 m 也磨掉。
        # 只在「障碍确实在车尾正后方」时才用这一档，避免倒车弧上被侧面邻车误拦。
        if gear < 0 and remaining > 2.0 and at is not None:
            lon, lat, _r = at
            behind = (lon < -HALF_L) and (abs(lat) < HALF_W + 0.35)
            # [同学 C 本轮修] 这一档原来是**绝对门限 0.42 m**，结果在「本来就贴着
            # 邻车揉库」的合法场景里每拍都触发：真机 W5 卡在 lon=−2.42 / lat=+0.9
            # （就是自己车尾左角对着邻车），日志里「倒车被后方障碍挡住」刷了 200 多
            # 行，每次都触发「进度停滞 → 重规划 → 立刻又被挡」的死循环。
            # 现在改成**相对规划余量**：规划器对这一条路径承诺的最小余量是
            # `self._plan_clear`，只有实测比它再少 0.08 m 以上才说明「执行跑偏了」。
            # 这样「规划本来就只给 0.18 m」的紧凑解不会再被自己拦死。
            # [B3 修复] 绝对上限 0.42 → 0.30。真机 B3 本轮：规划承诺车体余量
            # **+0.93 m**，实测倒车弧上车尾到邻车只剩 **0.41 m**（碰点
            # lon=−2.56 lat=−1.05），差 0.01 m 没够着 0.42 的门限 → 每拍停住
            # → 进度停滞 → 重规划（新解几何一样）→ 又被挡，8 s 内把 5 次微调
            # 预算烧光并报 INCOMPLETE。0.41 m 其实**远高于验收门限 0.20 m**，
            # 却被一道比验收严一倍的护栏判死，属于护栏自己把合法解拦死。
            # 收到 0.30 之后：仍能在「真跑偏」（<0.30）时及时停车，真兜底由
            # 下面 0.12 m 的主判据负责，不会因此放开到能剐蹭。
            rear_limit = min(0.30, max(0.10, self._plan_clear - 0.08))
            if behind and gap < rear_limit:
                self.stop()
                self._side_hits += 1
                if now - self._side_log > 1.0:
                    self._side_log = now
                    self.get_logger().warn(
                        f'倒车被后方障碍挡住：车体只剩 {gap:.2f} m'
                        f'（< {rear_limit:.2f} m），'
                        f'而路径还剩 {remaining:.1f} m —— 执行轨迹偏离规划，停车等重规划'
                        f'（碰点 lon={lon:+.2f} lat={lat:+.2f}）')
                return False
        if gap >= limit:
            self._escape_ref = gap
            self._escape_left = ESCAPE_SECONDS
            return True
        # [同学 C 本轮重写] 「已经在接触区里」的放行条件从「没更近」改成
        # **「真的在变远」**。
        #
        # 旧写法 `gap >= escape_ref − 0.03` 的后果是：自车一边顶着**会跟着动的
        # 干扰车**往后倒，一边间隙保持 0.06~0.07 m 不变 —— 判据认为「没更近」，
        # 于是整段都放行，车就把黄车一路推着走（真机日志「放行脱困」刷了 656 拍）。
        # 物理上「间隙不变」有两种可能：① 车在缓慢挪出夹角（该放行）；
        # ② 车正在推着别的东西走（必须停）。旧判据分不开这两者。
        #
        # 现在要求间隙**上升**才放行：挪出夹角时间隙必然变大；推着别人走时
        # 间隙恒定，直接判停。挪不动了就交给上层的「横向偏差持续 / 进度停滞」
        # 去重规划，不会再出现「相撞后继续加油门」。
        rising = gap > gap_prev + 0.002
        if self._escape_left > 0.0 and rising:
            self._escape_left -= dt
            self._escape_hits += 1
            if now - self._side_log > 1.0:
                self._side_log = now
                self.get_logger().warn(
                    f'入位已在接触区（{gap:.2f} m）但正在变远，放行脱困'
                    f'（本段还剩 {self._escape_left:.1f} s，累计 {self._escape_hits} 拍）')
            return True
        self.stop()
        self._side_hits += 1
        if now - self._side_log > 1.0:
            self._side_log = now
            hit = (f'lon={at[0]:+.2f} lat={at[1]:+.2f} r={at[2]:.2f}'
                   if at else '无命中点')
            self.get_logger().warn(
                f'入位贴合护栏：车体到最近障碍只剩 {gap:.2f} m（< {limit:.2f}，'
                f'{"变远" if rising else "未变远"}），停车（累计 {self._side_hits} 次，{hit}）')
        return False

    def _hit_dynamic(self, at):
        """命中点是否落在**动态代理**（干扰车 / 行人）身上。

        只用于「泊车质量验收」这一个用途：干扰车和行人是会走的，它们从车旁
        擦过时的 0.05~0.15 m 不代表自车剐了邻车。安全护栏（`_side_guard_ok`
        的停车判据、`_guard_ok`）**照旧对它们生效**，只是不参与 `min_park_gap`
        的统计。`at` 是 `body_clear` 给出的车体系命中点 (lon, lat, r)。
        """
        if at is None or self.pose is None:
            return False
        lon, lat, _r = at
        x, y, th = self.pose
        c, s = math.cos(th), math.sin(th)
        wx = x + c * lon - s * lat
        wy = y + s * lon + c * lat
        for pos in (self._agents or {}).values():
            if pos is not None and math.hypot(
                    pos[0] - wx, pos[1] - wy) < AGENT_HIT_MATCH + 1.0:
                return True
        for (px, py) in self.peds or []:
            if math.hypot(px - wx, py - wy) < 3.0:
                return True
        return False

    def _on_agent(self, msg, topic):
        p = msg.pose.pose.position
        self._agents[topic] = (p.x, p.y)

    def _agent_hold(self, now, cmd):
        """[B3 修复] 入位段停滞若由外部干扰车挡路造成 → 冻结等待，不烧微调预算。

        判据：侧向护栏不放行，**且**护栏命中点（车体到最近障碍的点）落在某台
        干扰车身上 —— 挡路的是「会倒退让行」的车，不是静态邻车/墙。这时停滞
        的责任不在自车规划，重规划救不了（新路径照样被它挡着），微调预算应该
        留给真正几何不可行的情况。等待期间：
          · 每帧刷新护栏：间隙恢复放行即解除冻结，交给原逻辑继续走（干扰车
            倒退让开的瞬间间隙「变远」，正好复用 _side_guard_ok 的放行条件）；
          · 顺延跟踪超时基点（120 s 只计「自己在走」的时间）；
          · 上限 AGENT_FREEZE_MAX，超时转回常规停滞处理，不会永久冻结。
        """
        guard_ok = self._side_guard_ok(now, gear=cmd.gear,
                                       remaining=cmd.remaining)
        if guard_ok or not self._agents:
            self._freeze_t0 = None
            return False
        hit = self._side_at
        x, y, th = self.pose
        matched = False
        if hit is not None:
            lon, lat, _r = hit
            wx = x + math.cos(th) * lon - math.sin(th) * lat
            wy = y + math.sin(th) * lon + math.cos(th) * lat
            for pos in self._agents.values():
                if pos is None:
                    continue
                if (math.hypot(pos[0] - wx, pos[1] - wy) < AGENT_HIT_MATCH
                        or math.hypot(pos[0] - x, pos[1] - y) < AGENT_NEAR_EGO):
                    matched = True
                    break
        if not matched:
            self._freeze_t0 = None
            return False
        if self._freeze_t0 is None:
            self._freeze_t0 = now
            self.get_logger().warn(
                f'停滞由外部干扰车挡路造成（车体间隙 {self._side_gap:.2f} m），'
                f'冻结等待其让路（上限 {AGENT_FREEZE_MAX:.0f} s，'
                f'不消耗微调预算）')
        self._follow_start = now      # 等待期间不累计 120 s 跟踪超时
        if now - self._freeze_t0 > AGENT_FREEZE_MAX:
            self.get_logger().warn(
                f'冻结等待干扰车让路 {now - self._freeze_t0:.0f} s 超时，'
                f'转常规停滞处理')
            self._freeze_t0 = None
            return False
        if now - self._freeze_log > 5.0:
            self._freeze_log = now
            self.get_logger().info(
                f'冻结等待干扰车让路中（{now - self._freeze_t0:.0f} s，'
                f'车体间隙 {self._side_gap:.2f} m，微调预算 '
                f'{self._dock_retry}/{DOCK_RETRY_MAX}）')
        return True

    def _guard_ok(self, cmd, now, ahead=None):
        """给即将下发的速度过一遍护栏；被拦则停车并返回 False。"""
        if abs(cmd.v) < 1e-3:
            return True
        # 末段 0.6 m 放行：车位纵深 5 m、车长 4.35 m，停到中心时车尾离后端
        # 障碍只有 0.33 m，比护栏余量还小——不放行的话「停到位」这一步会被
        # 自己的护栏挡住。这一段速度已经压到 0.24 m/s，且规划器保证无碰撞。
        if ahead is None and getattr(cmd, 'remaining', 9e9) < 0.6:
            return True
        if not self._lidar_guard(cmd.gear, ahead=ahead):
            self._guard_hits = 0
            self._guard_since = None
            return True
        self.stop()
        self._guard_hits += 1
        if self._guard_since is None:
            self._guard_since = now
        if now - self._guard_log > 1.0:
            self._guard_log = now
            extra = ''
            # 连续被拦超过 5 s 就把「拦的是什么」打出来：只报一句「有障碍」时，
            # 现场只能看到车不动、日志刷屏，分不清是行人、干扰车还是墙（实测
            # 出现过 300 s 的僵持，靠这条才定位到是前车不走的连锁）。
            if now - self._guard_since > 5.0:
                extra = '，' + self._guard_detail(cmd.gear)
            self.get_logger().warn(
                f'激光护栏：车体走廊内有障碍，停止'
                f'（{"前进" if cmd.gear > 0 else "倒车"}，累计 {self._guard_hits} 次，'
                f'已连续 {now - self._guard_since:.0f} s{extra}）')
        return False

    def _guard_detail(self, gear):
        """护栏命中点 + 最近行人的（行驶坐标系）位置，用于诊断僵持。"""
        out = []
        if self.scan is not None and self.pose is not None:
            ranges, amin, inc = self.scan
            best = None
            for i, r in enumerate(ranges):
                if not (0.06 < r < 6.0):
                    continue
                a = amin + inc * i
                lon = r * math.cos(a) + LIDAR_DX
                lat = r * math.sin(a)
                if gear < 0:
                    lon = -lon
                if -0.20 < lon < 4.5 and abs(lat) < HALF_W + GUARD_M:
                    if best is None or lon < best[0]:
                        best = (lon, lat, r)
            if best is not None:
                out.append(f'最近命中点 lon={best[0]:.2f} lat={best[1]:+.2f} '
                           f'r={best[2]:.2f}')
            else:
                out.append('最近命中点=无')
        else:
            out.append('无扫描')
        # 最近行人（map 帧换算到行驶坐标系）：距离 3 m 内才提，避免噪声
        if self.peds and self.pose is not None:
            th = self.pose[2] + (math.pi if gear < 0 else 0.0)
            c, s = math.cos(th), math.sin(th)
            best = None
            for (px, py) in self.peds:
                dx, dy = px - self.pose[0], py - self.pose[1]
                lon = c * dx + s * dy
                lat = -s * dx + c * dy
                d = math.hypot(max(0.0, abs(lon) - HALF_L),
                               max(0.0, abs(lat) - HALF_W))
                if best is None or d < best[0]:
                    best = (d, lon, lat)
            if best is not None and best[0] < 6.0:
                out.append(f'最近行人 距车体 {best[0]:.2f} m '
                           f'(lon={best[1]:+.2f} lat={best[2]:+.2f})')
            elif best is not None:
                out.append(f'最近行人 距车体 {best[0]:.1f} m（都在 6 m 外）')
        return '；'.join(out)

    def _creep_ok(self, gear):
        """[B3 修复] 蠕行跟车的前置检查：行进方向正前方窄条里最近障碍 > 2.5 m。

        只看 `|lat| < 半宽 + 0.2` 的窄条（与 corridor_hit 的远挡条件一致：
        远处能挡路的必然在这个窄条里），2.5 m 给 0.15 m/s 蠕行速度留足刹车
        余量。窄条外的东西（路缘、旁车道停驻车）不参与判距，不会把蠕行锁死。
        """
        if self.scan is None or self.pose is None:
            return False
        ranges, amin, inc = self.scan
        near = float('inf')
        for i, r in enumerate(ranges):
            if not (0.06 < r < 6.0):
                continue
            a = amin + inc * i
            lon = r * math.cos(a) + LIDAR_DX
            lat = abs(r * math.sin(a))
            if gear < 0:
                lon = -lon
            if lon > 0.1 and lat < HALF_W + 0.2:
                near = min(near, lon)
        return near > 2.5

    def on_risk(self, msg):
        v = msg.data.strip().lower()
        if v in ('safe', 'warning', 'danger'):
            self.risk = v
            self.risk_stamp = time.time()

    def on_driver_status(self, msg):
        self.driver_state = msg.data.strip().lower()
        self.driver_stamp = time.time()

    # ------------------------------------------------------------------ 主循环
    def on_timer(self):
        now = time.time()
        stale = (now - self.pose_stamp) > 1.0

        if self.state == 'WAIT_DATA':
            if self.pose is not None and self.scan_count >= int(
                    self.get_parameter('warmup_scans').value):
                self.state = 'SELECT_SPOT'
            else:
                self.publish_mode('idle')
            self.publish_status()
            return

        if bool(self.get_parameter('wait_for_driver').value) \
                and self.driver_state == 'running' and (now - self.driver_stamp) < 1.0:
            self.publish_mode('idle')
            self.publish_status()
            return

        if self.state == 'SELECT_SPOT':
            self.publish_mode('idle')
            self.stop()
            if now < self.retry_at:
                self.publish_status()
                return
            self._select_and_route(now)
            self.publish_status()
            return

        if self.state in ('PLAN',):
            self.publish_mode('idle')
            self.stop()
            # 规划解法离谱（换挡过多）时的定时重试，见 _plan 里的说明
            if self._plan_wait_until is not None and now >= self._plan_wait_until:
                self._plan_wait_until = None
                self._plan(now)
            self.publish_status()
            return

        if self.state in ('ROUTE', 'ALIGN', 'FOLLOW'):
            self.publish_mode('planner')
            if stale:
                self.stop()
                # [同学 C 修复] 位姿瞬断期间**不能算进路线行驶时间**。
                # 原来的超时只按 `now - _follow_start` 算，而 stale 时车是被强制
                # 停住的：一次 TF/SLAM 抖动就能让计时白跑十几秒，最后报
                # 「全局路线行驶超时，放弃」，接着重规划路线的起点落在最近的车道
                # 节点上、与自车差 1.5~2.6 m，横向偏差直接爆掉（实测真机 W4
                # 一轮里连报 4 次超时，路线彻底跑偏）。
                if self._pause_since is None:
                    self._pause_since = now
                self.get_logger().warn('位姿超时，暂停输出')
                self.publish_status()
                return
            if self._pause_since is not None:
                self._pause_total += now - self._pause_since
                self._pause_since = None
            if self.state == 'ROUTE':
                self._route_tick(now)
            elif self.state == 'ALIGN':
                self._align_tick(now)
            else:
                self._follow(now)
            self.publish_status()
            return

        self.publish_mode('idle')
        self.stop()
        self.publish_status()

    # ------------------------------------------------------------------ 选位
    def _select_and_route(self, now):
        forced = self.get_parameter('goal_stall').value.strip()
        if forced:
            stall = self.layout.find_stall(forced)
            if stall is None:
                self.get_logger().error(f'指定车位 {forced} 不存在，可用：'
                                        f'{self.layout.stall_ids()}')
                self.state = 'FAILED'
                return
            info = self.detector.evaluate_stall(stall)
            if not info['is_free']:
                self.get_logger().warn(
                    f'指定车位 {forced} 检测为 {info["state"]}，仍按指令尝试')
            chosen, chosen_info = stall, info
        else:
            region = self.get_parameter('stall_region').value.strip()
            chosen, chosen_info = self.detector.select(
                self.pose, exclude=self.blacklist, prefer=self.prefer,
                region=region or None)
        if chosen is None:
            # 自动选位：自车可能还在主路上，看不到场内车位。
            # 这时先沿车道图开到停车场入口，边开边用激光积累车位占用证据。
            entry = transform_point((0.0, 3.0), *self.spawn)
            if _dist(self.pose, entry) > 8.0:
                route = self.route_graph.plan(self.pose, (entry[0], entry[1], 0.0),
                                              start_heading=self.pose[2])
                if route:
                    self.recon = True
                    self.target = None
                    self._begin_route(route, now)
                    self.get_logger().info(
                        f'暂未看到可用车位，先沿车道图开往停车场入口'
                        f'（{self._route_len:.0f} m / {len(route)} 点）')
                    return
            self.get_logger().warn('暂未找到空闲车位，2 秒后重试', throttle_duration_sec=2.0)
            self.retry_at = now + 2.0
            return

        self.target = chosen
        self.recon = False
        # 一次泊车尝试开始：最小车体间隙从「无穷」重新统计（中途重规划不清零，
        # 验收看的是**整段**入位过程的最小值，见 MIN_PARK_GAP / _finish）
        self.min_park_gap = float('inf')
        self._dock_retry = 0
        self._end_retry = 0
        self._stuck_hits = 0
        self._stuck_t = 0.0          # [B3 修复] 停滞重规划的时间节流起点
        self._route_back_t0 = None   # [B3 修复] 本次倒车脱困的起始时刻
        self._lat_bad = 0
        self._route_lat_bad = 0.0
        self._replan_left = 2
        self._no_progress = 0.0
        self._reenter = False
        self._recover_left = RECOVER_MAX
        # [§6.36] 换目标就重新武装「退回路线重接入」的次数与 ALIGN 护栏计数，
        # 否则一次失败会把这两个预算永久耗掉，后续车位再也没机会重接入。
        self._reapproach_left = ALIGN_REAPPROACH_MAX
        self._align_guard_n = 0
        # [B3 修复] 换车位 = 新一轮泊车尝试：重新给满「路线段倒车脱困」次数
        # 与计时状态（`_begin_route` 只清计时、不清次数，见那里的说明）。
        self._route_back_n = 0
        self._route_back_from = None
        self._route_blk_anchor = None
        self._route_blk_t = 0.0
        # 换了目标就重新武装「到点对齐」的次数（_begin_route 不会重置它，
        # 否则对齐循环会把自己无限续命）
        self._aim_left = DOCK_AT_STAGE_RETRY
        # 换了目标就重新开窗：局部栅格要围绕「实际交接点 + 新目标」建，
        # 不能沿用上一次（甚至可能是自车初始位置）的窗口。
        self.planner = None
        self.get_logger().info(
            f'选定车位 {chosen.id}（{chosen.kind}/{chosen.region}）：'
            f'目标 {tuple(round(v, 2) for v in chosen.goal_pose)}，'
            f'状态 {chosen_info["state"]}，free_ratio={chosen_info["free_ratio"]:.2f}')
        self._publish_target_spot(chosen)

        staging = chosen.staging_pose
        # [同学 C 本轮] **在线重算倒车预备位姿**（全托管）。
        # 离线表 `config/staging_poses.yaml` 是拿「假定邻车 + 静态地图」算的，
        # 场上多一台干扰车、或者那个车位本来就不在表里（21 个垂直车位只有 11 个
        # 在表里，其余回退到「车位正前方 5.5 m 的同轴位姿」），位姿就不合适了。
        # 这里在**选中车位的这一刻**，用实时占用栅格重新搜一遍：
        # 沿过道中心线枚举 (半径 × 侧向 × 航向)，要求该位姿本身 free、
        # 且从它能 0 换挡开进车位、余量 ≥0.30 m；取余量最大的那个。
        # 搜不到才退回离线表（不会比现在更差）。
        if chosen.kind == 'perpendicular' and str(chosen.id)[:1] in ('W', 'E'):
            try:
                if self.planner is None:
                    self._build_local_planner()
                best = search_staging(
                    self.planner, chosen, approach_yaw=-math.pi / 2.0,
                    lane_x=None,
                    along=float(self.get_parameter('staging_along').value),
                    r_list=(5.74, 6.39, 7.00), min_clear=0.30, max_sw=0)
                if best is not None:
                    _, pose, c, ln = best
                    moved = _dist(pose, staging)
                    chosen.set_staging(pose)
                    staging = pose
                    self.get_logger().info(
                        f'预备位姿在线重算：{tuple(round(v, 2) for v in pose)}'
                        f'（相对离线表移动 {moved:.2f} m，计划余量 {c:+.2f} m，'
                        f'{ln:.1f} m / 0 换挡）')
                else:
                    self.get_logger().warn(
                        '在线搜索没找到合法预备位姿，沿用离线表的值')
            except Exception as exc:      # 搜索失败绝不能影响主流程
                self.get_logger().warn(
                    f'在线重算预备位姿失败（{exc}），沿用离线表的值')
        d = _dist(self.pose, staging)
        if d > ROUTE_THRESHOLD:
            route = self.route_graph.plan(self.pose, staging,
                                          start_heading=self.pose[2])
            if route:
                route = self._extend_to_staging(route, staging)
                self._begin_route(route, now)
                self.get_logger().info(
                    f'生成全局路线：{self._route_len:.1f} m，共 {len(route)} 个点，'
                    f'先开到预备位姿 {tuple(round(v, 2) for v in staging)}')
                return
            self.get_logger().warn('全局路线规划失败，尝试直接做局部规划')
        self._plan(now)

    def _extend_to_staging(self, route, staging):
        """把路线端点**对齐到预备位姿**：不够就补一段直道，冲过了就截断。

        [同学 C 本轮] 车道图上场内过道只有 3 个节点（间隔 7~8 m），而 11 个垂直
        车位的预备位姿落在它们之间，实测最大差 **5.64 m**（E7）；反过来 W5/E0/E3/E6
        又是**冲过了** 2.6~3.1 m（最后一个节点在预备位姿南边，路线开过头再想回来）。
        两种偏差都会让 `_plan` 的「锚定」判据（≤1.2 m）不成立，只好从过道中段
        规划 —— 离线量过，那样出来的弧**计划真实间隙只有 +0.05 m**（锚定时 +0.35）。

        为什么不直接往 route_graph.yaml 里硬写节点：预备位姿表是
        `compute_staging.py` 生成的，两份手写坐标一旦不同步就会静默失败
        （本轮已经在 `staging_poses.yaml` 的坐标系上栽过一次）。这里按几何做，
        永远跟着表走。

        判据：预备位姿到路线折线的最近点要**在过道中心线上**（横向 ≤0.6 m），
        且沿路线的弧长离终点 ≤6 m。补/截出来的那一段就是过道中心线的延长线，
        不会切角。
        """
        if not route or len(route) < 2:
            return route
        # ---- ⓪ 先单独判「路线末端 → 预备位姿」这一段 ----
        # [同学 C 本轮修复] 这一段的判据不能依赖垂足搜索：路线里常有零长度段 /
        # 末节点重复，垂足可能落在倒数第二段上，`i` 就对不上末端，`d_lat` 于是
        # 退化成「还差多远」而不是「偏了多少」。直接看端点几何最稳：
        # 只要「预备位姿 − 路线末点」这个向量**沿末端航向、法向偏差 ≤0.6 m**，
        # 就说明车还没开到预备位姿，补一段直道即可。
        # 实测 E6：路线停在 (53.0, −19.65)，预备位姿 (53.0, −17.165)，x 完全一致、
        # 只差 2.49 m。旧判据把它当成「偏了 2.49 m」直接放弃补齐 → 车提前 2.49 m
        # 交接 → 倒车弧整体前移 → 扫进 E5。W/E 两排普遍短 1~3 m，属共性问题。
        ex = staging[0] - route[-1][0]
        ey = staging[1] - route[-1][1]
        end_gap = math.hypot(ex, ey)
        if end_gap > 0.05:
            th_end = math.atan2(route[-1][1] - route[-2][1],
                                route[-1][0] - route[-2][0])
            seg_end = _dist(route[-1], route[-2])
            if seg_end < 1e-6:                 # 末段零长度：用存下来的 yaw
                th_end = route[-1][2]
            perp = abs(-math.sin(th_end) * ex + math.cos(th_end) * ey)
            along = math.cos(th_end) * ex + math.sin(th_end) * ey
            if perp <= 0.6 and 0.0 < along <= 8.0:
                out = list(route) + [(float(staging[0]), float(staging[1]),
                                      float(staging[2]))]
                self.get_logger().info(
                    f'补齐「路线末端 → 预备位姿」{end_gap:.2f} m 直段'
                    f'（法向偏差 {perp:.2f} m，末端航向已对齐）')
                return out
            # [同学 C §6.36] 法向偏差 0.6~2.2 m：补直段会在终点前拐一个折角，
            # 跟车器跟不动；改为在末端 `STAGE_BLEND_LEN` 米内做 S 形平滑横移。
            # 平行位（B/P 排）全都是这个量级：车道中心 y=-35.8、预备位姿 y=-34.70。
            blended = self._blend_to_staging(route, staging)
            if blended is not None:
                self.get_logger().info(
                    f'路线末端 S 形接入预备位姿：横向修正 {perp:+.2f} m、'
                    f'纵向 {along:+.2f} m，混合段 {STAGE_BLEND_LEN:.0f} m'
                    f'（原做法会带着这点残差交接 → ALIGN 擦邻车）')
                return blended
        # ---- ① 找预备位姿在路线上的垂足（弧长）----
        best = None            # (横向距离, 段下标, 段内 t, 累计弧长)
        acc = 0.0
        for i in range(len(route) - 1):
            a, b = route[i], route[i + 1]
            dx, dy = b[0] - a[0], b[1] - a[1]
            seg2 = dx * dx + dy * dy
            seg_len = math.sqrt(seg2)
            if seg2 < 1e-12:
                continue
            t = ((staging[0] - a[0]) * dx + (staging[1] - a[1]) * dy) / seg2
            t = max(0.0, min(1.0, t))
            px, py = a[0] + dx * t, a[1] + dy * t
            d = math.hypot(staging[0] - px, staging[1] - py)
            if best is None or d < best[0]:
                best = (d, i, t, acc + seg_len * t)
            acc += seg_len
        total = acc
        if best is None:
            return route
        d_lat, i, t, s_star = best
        overshoot = total - s_star
        # [同学 C 本轮修复] `d_lat` 是「预备位姿到路线折线的最近距离」。当路线
        # **还没开到**预备位姿时，垂足落在路线的最后一个点上，这个距离等于
        # 「还差多远」，而不是「横向偏了多少」。旧判据 `d_lat > 0.6 就放弃」
        # 于是把「差 2.5 m 没开到」误判成「预备位姿不在过道中心线上」，直接
        # 放弃补齐 —— 车就提前 2.5 m 交接。
        # 实测 E6：路线停在 (53.0, −19.65)，预备位姿在 (53.0, −17.165)，
        # 两者 x 完全相同（都在过道中心线上），只是 y 差 2.49 m。车于是在预备
        # 位姿**南边 2.49 m** 交接，规划出来的倒车弧整体前移 2.49 m，圆弧贴着
        # 车位口就转，车尾扫进 E5 —— 这正是用户看到的「从错误的一侧倒入、
        # 撞左边那台车」。W/E 两排的路线末节点普遍比预备位姿短 1~3 m，所以
        # 这不是 E6 一个车位的问题。
        # 修法：垂足落在路线末端时，把「沿末端方向还差多远」和「法向偏多少」分开，
        # 只对法向分量设 0.6 m 的门限。
        d_perp = d_lat
        if i >= len(route) - 2 and t > 0.985 and len(route) >= 2:
            th_end = math.atan2(route[-1][1] - route[-2][1],
                                route[-1][0] - route[-2][0])
            gx, gy = staging[0] - route[-1][0], staging[1] - route[-1][1]
            d_perp = abs(-math.sin(th_end) * gx + math.cos(th_end) * gy)
        if d_perp > 0.6 or overshoot > 6.0:
            return route           # 预备位姿不在路线附近：不动它
        if overshoot > 0.15:
            # ---- 冲过了：在垂足处截断，再接上预备位姿 ----
            # [同学 C] 必须先确认「截断后的到达方向」与预备位姿一致。
            # 东排（E0/E3/E6）就是反例：路线最后一个节点在预备位姿**南边**，
            # 车是朝南开到那里的、最后再原地转成朝北；如果直接截断，路线会
            # 变成「朝南到达朝北的预备位姿」，跟车器在终点前掉头，实测直接
            # 反向（终点航向差 143°/167°）。这种情况走下面的「补一段直道」。
            seg_yaw = route[i][2]
            if i + 1 < len(route):
                seg_yaw = math.atan2(route[i + 1][1] - route[i][1],
                                     route[i + 1][0] - route[i][0])
            if abs(wrap(seg_yaw - staging[2])) <= math.radians(60.0):
                head = list(route[:i + 1])
                if t > 1e-6:
                    yaw_a, yaw_b = route[i][2], route[i + 1][2]
                    head.append((route[i][0] + (route[i + 1][0] - route[i][0]) * t,
                                 route[i][1] + (route[i + 1][1] - route[i][1]) * t,
                                 wrap(yaw_a + wrap(yaw_b - yaw_a) * t)))
                out = head + [(float(staging[0]), float(staging[1]),
                               float(staging[2]))]
                self.get_logger().info(
                    f'路线截断到预备位姿：去掉冲过的 {overshoot:.2f} m'
                    f'（车道图节点不够密）')
                return out
        # ---- 还差一点：顺着车头方向补一段直道 ----
        p, th = route[-1], route[-1][2]
        dx, dy = staging[0] - p[0], staging[1] - p[1]
        d = math.hypot(dx, dy)
        if d < 0.05 or d > 8.0:
            return route
        if abs(wrap(math.atan2(dy, dx) - th)) > math.radians(20.0):
            return route
        prev = route[-2]
        seg = _dist(p, prev)
        if seg > 1e-3:
            seg_yaw = math.atan2(p[1] - prev[1], p[0] - prev[0])
            if abs(wrap(seg_yaw - th)) > math.radians(20.0):
                return route
        out = list(route) + [(float(staging[0]), float(staging[1]),
                              float(staging[2]))]
        self.get_logger().info(
            f'补齐「过道节点 → 预备位姿」{d:.2f} m 直段（车道图节点不够密）')
        return out

    def _blend_to_staging(self, route, staging):
        """在路线**末端若干米内**做 S 形接入，让路线正好落在预备位姿上（§6.36）。

        背景：南侧道路的车道中心在 y=-35.8，而平行位的预备位姿在 y=-34.70，
        每个 B/P 车位的交接都自带 **1.10 m 横向残差**。旧代码的补齐判据要求
        法向 ≤0.6 m 才肯补直段，于是直接放弃 —— 车带着 1.1 m 偏差交接，ALIGN
        必须一边横移一边摆正，真机 B3 就是这么擦上 `parked_b2` 的。

        算法：以路线末端行进方向为 u 轴、左法向为 v 轴，把预备位姿分解成
        （along, perp）；对「距末端 ≤ STAGE_BLEND_LEN」的每个路线点施加平移
        `delta(s) = (along, perp) · f(s)`，`s` = 该点到末端的弧长 / 混合段长度，
        `f(s) = (1-s)²(1+2s)`（f(0)=1、f(1)=0，两端导数为 0）。

        为什么用这个权重：f(1)=0 保证与前半段路线 C1 连续（不出现折角，跟车器
        跟得动）；f(0)=1 让末端精确落在预备位姿上。1.10 m / 12 m 的峰值曲率
        约 6·1.1/12² = 0.046（R≈22 m），远低于车辆极限 5.74 m。

        返回 None = 偏差超出可修正范围（或路线太短），由调用方保持原路线不动。
        """
        if not route or len(route) < 3:
            return None
        ex = staging[0] - route[-1][0]
        ey = staging[1] - route[-1][1]
        seg = _dist(route[-1], route[-2])
        th_end = (math.atan2(route[-1][1] - route[-2][1],
                             route[-1][0] - route[-2][0])
                  if seg > 1e-6 else route[-1][2])
        cu, su = math.cos(th_end), math.sin(th_end)
        along = cu * ex + su * ey
        perp = -su * ex + cu * ey
        if abs(perp) > STAGE_BLEND_LAT_MAX or abs(along) > STAGE_BLEND_ALONG_MAX:
            return None
        if abs(perp) < 0.05 and abs(along) < 0.05:
            return None
        # 每个点到末端的弧长
        dist_from_end = [0.0] * len(route)
        for k in range(len(route) - 2, -1, -1):
            dist_from_end[k] = dist_from_end[k + 1] + _dist(route[k], route[k + 1])
        idx = len(route) - 1
        while idx > 0 and dist_from_end[idx] < STAGE_BLEND_LEN:
            idx -= 1
        span = dist_from_end[idx]
        if span < 2.0:                       # 路线太短，平滑不了
            return None
        out = [(p[0], p[1], p[2]) for p in route[:idx]]
        for k in range(idx, len(route) - 1):
            s = min(1.0, dist_from_end[k] / span)
            f = (1.0 - s) * (1.0 - s) * (1.0 + 2.0 * s)
            du, dv = along * f, perp * f
            out.append((route[k][0] + cu * du - su * dv,
                        route[k][1] + su * du + cu * dv,
                        route[k][2]))
        out.append((float(staging[0]), float(staging[1]), float(staging[2])))
        # 平移后重算航向（折线切线）；末点沿用预备位姿自己的航向
        for k in range(max(1, idx), len(out) - 1):
            if _dist(out[k], out[k - 1]) > 1e-6:
                out[k] = (out[k][0], out[k][1],
                          math.atan2(out[k][1] - out[k - 1][1],
                                     out[k][0] - out[k - 1][0]))
        out[-1] = (out[-1][0], out[-1][1], float(staging[2]))
        return out

    # ------------------------------------------------------------------ 路线
    def _begin_route(self, route, now):
        self.route = route
        self.path = route
        self._route_len = sum(_dist(a, b) for a, b in zip(route, route[1:]))
        self._route_lat_bad = 0.0
        self._no_progress = 0.0
        self.follower = self.follower_route
        self.follower.set_path(route)
        self.state = 'ROUTE'
        self._follow_start = now
        self._pause_since = None
        self._pause_total = 0.0          # 新路线：重新开始计「有效行驶时间」
        self._yield_total = 0.0          # 让行等待也归零
        self._danger_total = 0.0         # 累计遮挡计时归零
        self._guard_escape_n = 0         # 护栏僵持脱困次数（见 _route_tick）
        self._route_remaining = 1e9
        # [B3 修复] 「原地被挡」计时随新路线重来；**倒车次数不清零**
        # （它属于这一次泊车尝试的预算，在 `_plan` 选新车位时才重置，
        #   否则每重规划一次就能再倒 4 次，等于没有上限）。
        self._route_blk_anchor = None
        self._route_blk_t = 0.0
        self._route_back_from = None
        self._route_back_t0 = None
        self.publish_path()

    def _route_tick(self, now):
        # 超时按路线长度给：最坏情况（自车在街区最东端、目标在西侧街）合法绕行要
        # 280 m，按 1.2 m/s 也要 4 分钟，写死 180 s 会把还没走完的路线判死。
        # [本轮] 150 → 180、0.5 → 0.45：再留一档余量。P1 a3 的实测账是
        # 「墙钟 377 s = 有效 287.8 s + 让行 41 s + 位姿瞬断 48 s」，正好卡在
        # 旧的 287.8 s 上；放宽后 68.9 m 的路线有 333 s 有效时间。
        limit = 180.0 + self._route_len / 0.45
        # 扣掉位姿瞬断被强制停住的时间（见执行器里的 `_pause_total`）
        elapsed = now - self._follow_start - self._pause_total - self._yield_total
        if elapsed > limit:
            self.get_logger().error('全局路线行驶超时，放弃')
            if self.target is not None:
                self.blacklist.add(self.target.id)
            self.target = None
            self.recon = False
            self.state = 'SELECT_SPOT'
            self.retry_at = now + 0.5
            self.stop()
            return
        # [同学 C 本轮修复] 这里是**路线阶段的死锁点**，原来只有 `stop(); return`。
        # risk_node 判 danger 的依据是「其它实体的**预测轨迹**在 1 s 内进入
        # 自车→目标车位的走廊」。自车一停下，这条走廊就完全静止了 ——
        # 站在旁边 3 m 处、只是"路过"走廊的行人也会被判 danger；
        # 而行人让行逻辑又是"看到车就等着/退 2 m"，两边互相等，谁都不动。
        # 真机日志（W4）连跑 3 次都在同一位置冻住 173 s，最后报
        # 「全局路线行驶超时，放弃」，而横向偏差只有 0.18 m —— 车根本没走偏。
        #
        # 处理：用**累计**遮挡时间（不是连续），超过 12 s 就切「低速蠕行」。
        # 不能用连续计时 —— 实测 danger 在行人身旁每 0.2~2 s 翻一次，
        # 连续计时永远被清零，12 s 的逃逸门限形同虚设（日志里一路
        # 「路线前方被遮挡，停车等待」而没有一次蠕行）。
        # 真的挡在必经之路上时由激光车体护栏（`_guard_ok`，4.5 m 前视）硬拦，
        # 蠕行不会撞上去；蠕行的作用只是把"预测轨迹擦过走廊"放过去。
        creeping = False
        if self.risk == 'danger':
            self._danger_total += 0.05
            self._yield_total += 0.05
            # [本轮] 12 → 6 s：动态场景里 danger 在行人/干扰车旁每 0.2~2 s 翻一次，
            # 12 s 的积累门限让自车「停 12 s → 蠕行几拍 → 又停 12 s」，
            # 真机 P1 三轮都是 68.9 m 的路线走了 180 s 才 47 m（danger 累计
            # 600~1300 次）。6 s 仍能滤掉真正的短时遮挡，但让车更早开始蠕行。
            if self._danger_total <= 6.0:
                if self.danger_since is None:
                    self.danger_since = now
                    self.get_logger().warn('路线前方被遮挡，停车等待')
                self.stop()
                return
            if self.danger_since is not None:
                self.danger_since = None
                self.get_logger().warn(
                    f'路线累计遮挡 {self._danger_total:.1f}s，低速蠕行通过')
            creeping = True
        else:
            self.danger_since = None
            # 遮挡解除后慢速放电，避免「偶尔擦一下」也累积成蠕行
            self._danger_total = max(0.0, self._danger_total - 0.10)
        scale = 0.5 if self.risk == 'warning' else 1.0
        if creeping:
            scale = min(scale, 0.35)
        # [同学 C 本轮] 路线**末段减速**：给航向环留出收敛距离。
        # 实测交接位姿的位置很准（0.1~0.4 m），但**航向常差 8~12°** ——
        # 1.4 m/s 冲过最后 10 m 只用了 7 s，航向环来不及把出弯留下的十几度收掉，
        # 而垂直车位的入位弧要求起始航向基本对准（差 12° 时离线在 UNDER≤0.70
        # 下横向偏差会从 0.10 m 涨到 1.0 m 以上并剐蹭）。
        # 末段 12 m 压到 ~0.63 m/s（scale 0.45），有 19 s 可以收敛。
        if self._route_remaining < 12.0:
            scale = min(scale, 0.45)
        cmd = self.follower.update(self.pose, 0.05, speed_scale=scale,
                                   yaw_rate=self._yaw_rate(now))
        self._route_remaining = cmd.remaining

        if self.recon:
            # 侦察模式：开到停车场入口后重新选位
            if cmd.done or cmd.remaining < 2.0:
                self.stop()
                self.recon = False
                self.state = 'SELECT_SPOT'
                self.get_logger().info('已到停车场入口，重新选位')
                return
        else:
            staging = self.target.staging_pose
            bad_lat = cmd.lateral_error > ROUTE_BAD_LAT
            self._route_lat_bad = self._route_lat_bad + 0.05 if bad_lat else 0.0
            if self._route_lat_bad > ROUTE_BAD_HOLD:
                # 偏差超限：如果离路线终点还远，说明只是出弯没咬住线，重新规划一条
                # 路线把车拉回车道即可 —— 直接把十几米外的位置丢给 Hybrid A* 去做
                # 揉库规划，既慢（场内 90° 转角附近要搜上万次扩展）又容易失败。
                self._route_lat_bad = 0.0
                if cmd.remaining > 6.0 and self._replan_left > 0:
                    self._replan_left -= 1
                    self.get_logger().warn(
                        f'路线跟踪偏差 {cmd.lateral_error:.1f} m，重新规划路线'
                        f'（剩余 {self._replan_left} 次）')
                    self.stop()
                    route = self.route_graph.plan(
                        self.pose, staging, start_heading=self.pose[2])
                    if route:
                        route = self._extend_to_staging(route, staging)
                        self._begin_route(route, now)
                        return
                self.get_logger().info(
                    f'交接到局部规划（横向偏差 {cmd.lateral_error:.1f} m），'
                    f'由 Hybrid A* 完成最后一段')
                self.stop()
                self._handoff(now)
                return
            # 交接条件：路线走完了。不用「距预备位姿小于 X 米」提前交接 ——
            # 路侧平行车位的预备位姿在车位**南侧** 5.2 m（倒车入位的起点），
            # 提前在 3.5 m 处停车等于把车停在车位正前方，反而要做一次多余的掉头。
            # 剩余距离给到 2.5 m：跟车器在终点前可能因为航向差太大而「既不算到位
            # 也不往前走」，停在 1 m 处死等（实测卡了 170 s）。
            self._no_progress = (self._no_progress + 0.05
                                 if abs(cmd.v) < 0.05 else 0.0)
            # 剩余距离给到 1.5 m：跟车器在终点前可能因为航向差太大而「既不算到位
            # 也不往前走」，停在 1 m 处死等（实测卡了 170 s），所以留一点余量。
            # [同学 C 调整] 原来写 2.5 m —— 那是**零预瞄**时代的补偿：当时垂足会
            # 滞后于车（甚至前跳），`remaining` 不准，2.5 m 实际让车停在离车道节点
            # 3 m 出头的地方。改成预瞄纯跟踪后垂足就在车正下方，`remaining` 准了，
            # 2.5 m 会让车**少走 0.7 m**，交接位姿离预备位姿 5.3 m（原来 4.65 m），
            # 于是「一把倒入」的直接解搜不出来，退化到需要 90° 原地转的两段式，
            # 实测那一轮连续 8 次规划失败。这里收到 1.5 m，保证真的开到车道节点。
            #
            # [同学 C 再调整] 1.5 m 仍然太松。离线用「真实交接位姿」复现
            # （`HANDOFF_BACK`）量过：交接位姿沿路线往回退 0.2 m 时 11 个可用
            # 垂直车位 0 剐蹭，退 0.4/0.6/0.8/1.0 m 时分别出现 1/1/1/3 个剐蹭，
            # 退 1.0 m 时 W7/E0/E3 的执行轨迹真实车体间隙掉到 **0.00 m**。
            # 原因是 2.5 m 车位的净余量只有 0.325 m，交接位姿每差 0.1 m 就直接从
            # 这点余量里扣。所以这里收到 0.25 m：要么真的开到路线终点，要么由
            # `_no_progress`（6 s 原地不动）兜底交接。
            # [同学 C 再调整] 交接阈值 0.25 → 0.10 m：路线跟车器的终点判据已经
            # 收紧到 0.20 m / 6°，0.25 m 会**先于**它触发，结果还是「差 0.4 m、
            # 歪 10° 就交接」（真机 W7 两轮都是这样）。收到 0.10 m 之后由 `done`
            # 主导；真的收敛不了还有 `_no_progress`（6 s 原地不动）兜底。
            # [同学 C 本轮] 试过「提前 3.2 m 交接，让两段式的第一段变成 3 m 直道」，
            # **离线证明更差**（`probe_lead.py`）：从过道中段规划，
            #   back=0    → 计划真实间隙 +0.35，执行后 +0.37（几乎无损）
            #   back=1.0  → +0.07，执行后 0.00（剐）
            #   back=2.0  → +0.05，执行后 0.00
            #   back=3.2  → 两段式 +0.35，但执行横向峰值 0.36~0.43（换挡瞬态），间隙 0.00
            # 也就是说「越早开始机动，几何越差」——因为预备位姿本身就是这条弧的
            # 切点，离开切点规划出来的都是更弯的 S。所以不提前交接，改成
            # 「机动锚定在预备位姿上」（见 `_plan` 里的 anchor）。
            # [同学 C 本轮] `_no_progress` 兜底交接必须**只在快到终点时**生效。
            # 它原本是给「快到预备位姿时被航向差卡住」用的；但行人在进场车道
            # 一站就是十几秒，6 s 不动就强制交接，会把车丢在离预备位姿 **17.4 m**
            # 的地方去做泊车规划 —— 真机实测规划出 31.47 m / 换挡 1 次 / 余量仅
            # +0.17 m 的怪解，然后越修越偏。远离终点时被挡就该继续等。
            # [B3 修复] 判据从「指令速度 <0.05 累计 6 s」改成 **`_route_blk_t`
            # （真实被挡时长）**，并把距离门限 3.0 → 6.0 m。
            # 旧判据看的是**指令速度**：护栏拦住时跟车器照样算出 v=0.63，
            # `_no_progress` 每拍被清零，于是「车明明一动不动、离终点只剩 4 m」
            # 也永远不触发强制交接（真机 B3 本轮：剩余 4.0 m 冻结了 40+ s，
            # 连续倒车脱困两次都回到同一处）。`_route_blk_t` 只在净位移
            # ≤1 m 时累加，是真正的「原地被挡」计时。
            stuck_near_end = (self._route_blk_t > 6.0 and cmd.remaining < 6.0)
            if cmd.done or cmd.remaining < 0.10 or stuck_near_end:
                if stuck_near_end and not (cmd.done or cmd.remaining < 0.10):
                    self.get_logger().warn(
                        f'快到预备位姿但 6 s 没动（剩余 {cmd.remaining:.1f} m），'
                        f'强制交接')
                # 「必须先准确到达预备位姿才入位」这个方案实测走不通，已回退：
                # 车道图的终点落在过道节点上，自车结束路线时距预备位姿
                # 4.6~4.7 m、航向差 **91~94°** —— 因为垂直车位的预备位姿本来
                # 就与过道正交（车头朝车道），它不是车道上能被「开到」的点，
                # 而是一个**机动起点**。尝试过两种对齐手段，都不成立：
                #   · 用车道图再规划：终点只到过道节点，规划出来只剩 2.2 m，
                #     下一拍仍判「不到位」，白耗一次机会；
                #   · 给跟车器一条「当前位姿→预备位姿」的插值路径：4.7 m 内
                #     要转 91°，等于要求 κ≈0.34 rad/m（车极限 0.20），跟车器
                #     直接卡住，最后报「全局路线行驶超时，放弃」。
                # 结论：对齐必须是一个**独立机动**（Hybrid A* 或圆弧），不能
                # 塞进路线跟踪里。见下方 _plan 的两段式，它现在就是干这个的。
                self.get_logger().info(
                    f'路线完成（剩余 {cmd.remaining:.1f} m），交接到局部规划'
                    f'（距预备位姿 {_dist(self.pose, staging):.1f} m / '
                    f'{math.degrees(abs(wrap(self.pose[2] - staging[2]))):.0f}°）')
                self.stop()
                self._handoff(now)
                return
        tw = Twist()
        tw.linear.x = float(cmd.v)
        tw.angular.z = float(cmd.omega)
        # 巡航段主动让行行人：激光护栏只能拦住「已经贴到车体」的情形，
        # 从外面看仍是「车顶着人往前开」，所以这里按行人位姿提前停。
        if self._ped_in_front():
            self.stop()
            self._yield_total += 0.05      # 让行等待不计入路线超时
            if now - self._ped_log > 1.5:
                self._ped_log = now
                self.get_logger().info('巡航让行：前方有行人，停车等待')
            return
        if not self._guard_ok(cmd, now,
                              ahead=float(self.get_parameter('guard_ahead').value)):
            self._yield_total += 0.05
            # [B3 修复 · 原地被挡计时] 用**路线剩余里程**当进度锚点，而不是
            # 车体位移。理由：护栏抖动时（真机 B1 每 2 s 交替「拦 / 放」一次）
            # 车会在原地来回蹭，位移一会儿就超过 1 m 把计时清零，可 `剩余`
            # 一直冻在 18.7 m —— 计时永远攒不到 20 s，倒车脱困一次都不触发。
            # `remaining` 只认「沿路线真的走近了」，是唯一不会骗人的进度量。
            if self._route_blk_anchor is None:
                self._route_blk_anchor = cmd.remaining
                self._route_blk_t = 0.0
            elif cmd.remaining < self._route_blk_anchor - ROUTE_BLOCK_RESET_DIST:
                self._route_blk_anchor = cmd.remaining
                self._route_blk_t = 0.0
            else:
                self._route_blk_t += 0.05
            # [B3 修复 · 路线段倒车脱困] 优先级最高：正在倒，就退够 ROUTE_BACK_DIST。
            if self._route_back_from is not None:
                done = math.hypot(self.pose[0] - self._route_back_from[0],
                                  self.pose[1] - self._route_back_from[1])
                # [B3 修复] 倒车同时受**时长**约束：SLAM 位姿跳变时
                # `hypot` 会被瞬间放大，实测出现过「倒车脱困 9.7 m 完成」
                # （应为 2.5 m）—— 一下退出去十几米，再开回来又堵在同一处，
                # 单次脱困白烧 30 s+，B3 因此连跑两轮路线超时。
                # 2.5 m / 0.30 m/s = 8.3 s，给 10 s 上限。
                timed_out = (self._route_back_t0 is not None
                             and now - self._route_back_t0 > 10.0)
                if done >= ROUTE_BACK_DIST or timed_out:
                    self._route_back_from = None
                    self._guard_since = None
                    self._route_blk_anchor = None
                    self._route_blk_t = 0.0
                    self.stop()
                    self.get_logger().warn(
                        f'倒车脱困 {done:.1f} m 完成（第 {self._route_back_n}/'
                        f'{ROUTE_BACK_MAX} 次），按新位姿重新规划路线')
                    route_b = self.route_graph.plan(
                        self.pose, self.target.staging_pose,
                        start_heading=self.pose[2])
                    if route_b:
                        route_b = self._extend_to_staging(
                            route_b, self.target.staging_pose)
                        self._begin_route(route_b, now)
                    return
                # 后方（车尾方向）必须干净。注意 `ahead` 是「从激光原点到车后
                # 多远」，激光装在车体中心附近而车尾还有半个车长（HALF_L≈2.18 m），
                # 所以要在「车尾外 1.2 m」这一档上加回 HALF_L，否则只看到车尾
                # 内侧 1.2 m，等于没检查，倒车会直接压上后面的东西。
                if not self._lidar_guard(-1, ahead=HALF_L + 1.2):
                    back = Twist()
                    back.linear.x = -0.30
                    back.angular.z = 0.0
                    self.pub_cmd.publish(back)
                    if now - self._guard_log > 3.0:
                        self._guard_log = now
                        self.get_logger().info(
                            f'倒车脱困中：已退 {done:.1f}/'
                            f'{ROUTE_BACK_DIST:.1f} m')
                else:
                    self._route_back_from = None
                    self.get_logger().warn('倒车脱困中止：车尾方向被占')
                return
            if (self._route_blk_t >= ROUTE_BACK_WAIT
                    and self._route_back_n < ROUTE_BACK_MAX):
                if not self._lidar_guard(-1, ahead=HALF_L + 1.2):
                    self._route_back_n += 1
                    self._route_back_from = self.pose[:2]
                    self._route_back_t0 = now
                    self.get_logger().warn(
                        f'原地被挡累计 {self._route_blk_t:.0f} s 且前路无解'
                        f'（车头对着障碍，静止无法转向），'
                        f'第 {self._route_back_n}/{ROUTE_BACK_MAX} 次倒车 '
                        f'{ROUTE_BACK_DIST:.1f} m 脱困')
                    back = Twist()
                    back.linear.x = -0.30
                    self.pub_cmd.publish(back)
                    return
                self.get_logger().warn(
                    f'原地被挡 {self._route_blk_t:.0f} s，但车尾方向被占，'
                    f'无法倒车脱困，继续等待')
                self._route_blk_t = 0.0
            # [B3 修复 · 蠕行跟车] 护栏被**远处**静止障碍顶住时别原地干等。
            # 4.5 m 前视把「3~4.5 m 处停着的干扰车」也当硬挡，自车一动不动，
            # 要么白等到路线超时、要么 45 s 后被迫交接/绕行。这里先低速
            # （0.15 m/s）跟近到 2.5 m（_creep_ok 用正前方窄条判距，留足刹车
            # 余量），真到跟前再停 —— 干扰车一动，护栏解除，立刻恢复巡航。
            if (self._guard_since is not None
                    and now - self._guard_since > 3.0
                    and self._creep_ok(cmd.gear)):
                creep = Twist()
                creep.linear.x = 0.15 if cmd.gear > 0 else -0.15
                self.pub_cmd.publish(creep)
                if now - self._guard_log > 5.0:
                    self._guard_log = now
                    self.get_logger().info(
                        '护栏远处静止障碍：低速蠕行跟近（0.15 m/s）')
            # [本轮修复 · 建议 5b] 路线段被同一片障碍**长期**顶住就主动脱困。
            # 真机 SW1：自车在 (-26.0, 9.9) 与干扰车互相让行，末端 100 帧位移
            # 0.000 m，白等 309 s 直到「全局路线行驶超时，放弃」。
            # 这种死锁是「干扰车前方 10.5 m 有车就停」+「自车 4.5 m 激光护栏」
            # 两端同时生效的必然结果，光让干扰车倒退不一定够，自车侧必须有出口：
            # 提前交接给局部规划（Hybrid A* 允许换挡，能绕过静止障碍）。
            if (self._guard_since is not None
                    and now - self._guard_since > ROUTE_GUARD_ESCAPE
                    and self._guard_escape_n < ROUTE_GUARD_ESCAPE_MAX):
                d_st = _dist(self.pose, self.target.staging_pose)
                held = now - self._guard_since
                self._guard_escape_n += 1
                self._guard_since = None
                self.stop()
                if d_st < ROUTE_GUARD_ESCAPE_DIST:
                    self.get_logger().warn(
                        f'路线被挡 {held:.0f} s 未解除，'
                        f'提前交接局部规划绕行（{self._guard_escape_n}/'
                        f'{ROUTE_GUARD_ESCAPE_MAX}）')
                    self._handoff(now)
                elif self._replan_left > 0:
                    # [本轮修复] 离目标还远时**不能**交接（Hybrid A* 的 ±15 m
                    # 窗口根本覆盖不到车位，平行位还会触发几十米的 ALIGN），
                    # 但也不能一直等：真机里这种「前方障碍」僵持最后都是
                    # 「全局路线行驶超时，放弃」。改为**重规划一条路线**，
                    # Dijkstra 会换一条（绕开被堵的那条街）继续走。
                    self._replan_left -= 1
                    route_r = self.route_graph.plan(
                        self.pose, self.target.staging_pose,
                        start_heading=self.pose[2])
                    if route_r:
                        route_r = self._extend_to_staging(
                            route_r, self.target.staging_pose)
                        self.get_logger().warn(
                            f'路线被挡 {ROUTE_GUARD_ESCAPE:.0f} s 且距目标 '
                            f'{d_st:.0f} m，重规划路线绕行'
                            f'（剩 {self._replan_left} 次）')
                        self._begin_route(route_r, now)
                        return
                    self.get_logger().warn(
                        '路线被挡且无备选路线，继续等待')
            return
        self.pub_cmd.publish(tw)
        if now - self._last_cmd_log > 3.0:
            self._last_cmd_log = now
            self.get_logger().info(
                f'路线行驶 v={cmd.v:.2f} 剩余={cmd.remaining:.1f} m '
                f'横向偏差={cmd.lateral_error:.2f} m 行人={len(self.peds)}')

    # ------------------------------------------------------------------ 规划
    def _strip_micro_head(self, res, goal):
        """把「起步小段」并掉：第一段比 `MIN_SEG_LEN` 还短时，从换挡点重规划一条
        干净曲线，再与**原起步头拼接**返回。

        [同学 C 修复] 原实现直接「替换」成从换挡点出发的新路径 —— 但车还站在
        起点上，新路径的第一个点悬在换挡点处，头部空出 0.3~0.6 m：第一拍投影
        落到新弧内部（真机 W9 实测 seg=2/t=0.60），φ 凭空 −14.3°、κ 顶死限幅、
        横向峰值 1.02 m（§6.23⑤2）。现在改为**拼接**：原起步头（车实际要走的那
        零点几米，起点就是车的位姿）+ 换挡点起的重规划弧；配合跟车器的**首换挡
        窄门限**（`controller.PathFollower.first_shift_arc`），这段头会被真的开出
        来，而不是被换挡门限原地跳过。重规划仍然值得做：原路径换挡点之后的尾巴
        可能还带着揉库（多换挡），从换挡点直连往往就是一条 0 换挡的干净弧。
        """
        if not res.success:
            return res
        path = list(res.path)
        i = next((k for k in range(1, len(path)) if path[k][3] != path[0][3]), None)
        if i is None:
            return res
        leg = sum(_dist(a, b) for a, b in zip(path[:i], path[1:i + 1]))
        if leg >= MIN_SEG_LEN:
            return res
        r = self.planner.plan(path[i][:3], goal)
        if r.success:
            merged = dedup_path(path[:i + 1] + list(r.path))
            if len(merged) >= 2:
                n_sw = sum(1 for a, b in zip(merged, merged[1:]) if a[3] != b[3])
                # [同学 C 修复] 门禁必须用**拼接后的实际换挡数**：旧写法检查
                # 重规划解自己的 `r.n_switch`，但拼接点可能再引入一次换挡
                # （头部末端档位 ≠ 新弧首点档位，R→D→R 抖动），导致「1 换挡的
                # 干净直连解」被「3 换挡的拼接解」顶掉 —— E2 在 plan_max_steer
                # 0.40 下就是这个形态：低曲率解析解明明存在，执行的是垃圾解。
                if n_sw <= res.n_switch:
                    rev = sum(_dist(a, b) for a, b in zip(merged, merged[1:])
                              if b[3] < 0)
                    total = leg + r.length
                    self.get_logger().info(
                        f'起步小段拼接：保留开头 {leg:.2f} m，从换挡点改走重规划弧'
                        f'（{r.length:.1f} m，共 {total:.1f} m / 换挡 {n_sw} 次）')
                    return PlanResult(True, merged, cost=total, reason='拼接',
                                      used_rs=True, length=total,
                                      reverse_length=rev, n_switch=n_sw)
        return res

    def _plan(self, now):
        self.state = 'PLAN'
        self.follower = self.follower_park
        goal = self.target.goal_pose
        stage = self.target.staging_pose
        # [同学 C 本轮 §6.29] 规划器是**后轴**参考：goal/stage/自车位姿进规划器
        # 前统一换算（`_rear`）。anchor/对齐/终点等**物理判据**仍用中心位姿 ——
        # 它们比的是真实车与真实车位，与参考点选取无关。
        goal_r = self._rear(goal)
        stage_r = self._rear(stage)
        t0 = time.time()
        # 新解就从「当前实际间隙」重新给一次脱困放行额度（见 _side_guard_ok）
        self._escape_left = ESCAPE_SECONDS
        self._escape_ref = float('inf')

        if self.planner is None:
            self._build_local_planner()

        res = None
        used_stage = False
        two_stage_par = False
        stage_tail = None      # 预备位姿 → 车位 那一段（直线或圆弧）
        if self.target.kind == 'perpendicular':
            # [同学 C 本轮] 「机动锚定在预备位姿上」。
            # 真机交接位姿总会差 0.4~0.5 m / 8~10°（路线末段有个小转角，跟车器
            # 收不干净）。**从这个偏位姿规划**出来的圆弧，几何就变形了：离线量到
            # 计划真实间隙从 +0.35 掉到 +0.07（`probe_lead.py`），执行后直接剐。
            # 所以偏差在 1.2 m / 12° 以内时，**规划起点直接用预备位姿**，
            # 路径永远是那条验证过的「0 换挡、余量 +0.55」的圆弧；自车带着小偏差
            # 去追这条弧，靠跟车器的反馈收敛（离线实测：从 0.5 m / 8° 出发，
            # 执行横向峰值 0.03~0.08 m，终点 0.15 m / 1°）。
            # [同学 C §6.30] **直线解车位**（staging 与 goal 同航向且共轴线，如
            # E0 表条目 [5.5,0,0] 的正前方直线倒车）把锚定航向门限放宽到 10°：
            # 弧线解 4° 门限的依据是「初始航向误差 Δψ 消耗曲率余量、误差与转弯
            # 方向相反时切进邻车」（probe_yaw_sweep.py：E3 +6° 间隙 +0.38→+0.12），
            # 而直线解 κ=0，不存在曲率余量问题，航向误差全由反馈吸收。离线
            # verify_park_sim 实测（E0，锚定后）：Δψ=4/6/8/10° 执行间隙
            # +0.41/+0.34/+0.27/+0.20，噪声 4 种子（8° 交接）全部 +0.27。
            # 不锚定则从偏位姿直连，E0 退成 3 换挡揉库解、间隙 0.00（剐蹭）。
            _gdx, _gdy = goal[0] - stage[0], goal[1] - stage[1]
            _straight_stage = (
                abs(wrap(stage[2] - goal[2])) < math.radians(1.0)
                and abs(-math.sin(stage[2]) * _gdx
                        + math.cos(stage[2]) * _gdy) < 0.1)
            _anchor_yaw_tol = (
                10.0 if _straight_stage
                else float(self.get_parameter('anchor_yaw_tol').value))
            # [同学 C §6.31] 位置门限从「欧氏 1.2 m」改成**沿 / 横分解，各 0.30 m**。
            # 真机 W7（唯一超差车位）是「航向 0°、纵向差 1.13 m」：欧氏距离把这段
            # 纯纵向缺口当成「就在预备位姿上」，锚定后跟踪首拍就有 1.13 m 的初始
            # 横向误差，把 0 换挡弧的 +0.30 m 余量一次吃穿。
            _st_along, _st_lat = stage_offset(self.pose, stage)
            anchor = (abs(_st_along) <= STAGE_XY_TOL
                      and abs(_st_lat) <= STAGE_XY_TOL
                      and abs(wrap(self.pose[2] - stage[2]))
                      < math.radians(_anchor_yaw_tol))
            # [同学 C 本轮] **起点非法保护**。
            # 真机 W5 的两次失败都伴随「规划出 26 m / 3 换挡 / 余量 −0.22 m」的解 ——
            # 负余量意味着路径本身穿障碍物。原因是自车已经在和占用栅格重叠
            # （贴着邻车、或被干扰车挤住），从非法起点出发 RS 解析解全部失效，
            # 只能给出可行域边缘的垃圾解，执行起来就是「在车位口揉三把磨掉 126 m」。
            # 处理：起点非法且离预备位姿不远时，**直接以预备位姿为起点**规划
            # （车带着几米偏差去追那条已验证的圆弧，靠反馈收敛）；
            # 预备位姿也非法就交给后面的「离谱解拒绝」去停等重试。
            if (not anchor) and not self.planner.free(*self._rear(self.pose)):
                d_st = _dist(self.pose, stage)
                if d_st < 4.0 and self.planner.free(*stage_r):
                    anchor = True
                    self.get_logger().warn(
                        f'自车位姿与占用栅格重叠（距预备位姿 {d_st:.1f} m），'
                        f'改以预备位姿为规划起点')
            plan_from = stage_r if anchor else self._rear(self.pose)
            if anchor:
                self.get_logger().info(
                    f'机动锚定在预备位姿：自车偏差 '
                    f'{_dist(self.pose, stage):.2f} m（沿 {_st_along:+.2f} / '
                    f'横 {_st_lat:+.2f} m）/ '
                    f'{math.degrees(abs(wrap(self.pose[2] - stage[2]))):.0f}°，'
                    f'规划起点改用预备位姿')
            direct = self._strip_micro_head(
                self.planner.plan(plan_from, goal_r), goal_r)
            if self.planner.last_start_relaxed:
                self.get_logger().warn(
                    f'起点与障碍膨胀区重叠，已放宽起点判据（'
                    f'{self.planner.start_free_radius:.2f} m 内不判碰撞）')
            res = direct if direct.success else None
            # [同学 C 修复] 「几把入库」由**换挡次数**决定，所以必须把直连解和
            # 两段式放在一起比，而不是「只要直连不是 0 换挡就退两段式」。
            #   · 直连 0 换挡 = 纯倒车一把倒入；
            #   · 直连 1 换挡 = 先前进到位、再一把倒入 —— 这也是正常的倒车入库；
            #   · ≥2 换挡才是多点揉库。
            # 旧判据（n_switch==0 才用直连）会把「1 换挡的直连解」丢掉，退到
            # 两段式；而两段式要求自车先摆成「垂直于车位、车头朝外」的预备位姿，
            # 等于要在 6 m 宽的通道里原地转 90°，实测要 3 次换挡 —— 这就是
            # 「多把入库、第一把倒在两个车位中间、擦着邻车右前角」的来源。
            # [同学 C 修复] 已经基本站在预备位姿上（≤0.8 m）时**不要**走两段式：
            # 「开到预备位姿」那一段只剩十几厘米，会变成一条 0.1~0.4 m 的起步
            # 小段，跟车器在换挡门限上直接跳过它、参考点落到倒车弧里，闭环发散。
            # 这种情形直连解本身就是那条验证过的 0 换挡圆弧（`_strip_micro_head`
            # 已把残留的小段并掉）。
            near_stage = anchor
            if ((not direct.success) or direct.n_switch >= 1) and not near_stage:
                r2 = self.planner.plan(self._rear(self.pose), stage_r)
                # [同学 C 修复] 预备位姿 → 车位 这一段不能再**强制走直线**。
                # 旧代码用 `sample_straight(stage, goal)`，这就把预备位姿锁死在
                # 「车位的正前方轴线上」—— 因为只有轴线上的点才能用直线倒进去。
                # 而轴线上的点要进库必须先原地转 90°，车的最小转弯半径 6.39 m
                # 在 12.5 m 过道里转不开，只能揉 2~3 把（用户看到的「永远不能
                # 一把入位、第一把倒在两车位中间」）。
                # 现在预备位姿改成「一个转弯半径之外的过道点」（见
                # `config/staging_poses.yaml`，由 `scripts/compute_staging.py`
                # 用 Hybrid A* 逐位验证 0 换挡），那一段自然就是「一段 90° 圆弧
                # + 一点直线」，所以这里优先用**规划出来的圆弧**，直线只作兜底。
                tail_res = self.planner.plan(stage_r, goal_r)
                tail = None
                if tail_res.success and tail_res.n_switch == 0:
                    tail = list(tail_res.path)
                elif self._straight_ok(stage_r, goal_r):
                    tail = sample_straight(stage_r, goal_r, 0.10)
                    tail_res = None
                if r2.success and tail is not None:
                    path2 = list(r2.path) + tail
                    if not direct.success:
                        res, used_stage, stage_tail = r2, True, tail
                        # 注意：res.path 里没有那一段，下面按 stage_tail 拼
                    else:
                        # [同学 C 修复] 选解规则从「换挡次数最少」改成**余量优先**。
                        # 离线用真实交接位姿复现过（`scripts/compare_handoff.py`）：
                        # W7 交接位姿横偏 0.25 m 时，
                        #   · 直接解 0 换挡、计划余量 +0.168 m → 执行后真实间隙 **0.00 m（剐）**；
                        #   · 两段式 1 换挡、计划余量 +0.550 m → 执行后真实间隙 **+0.45 m**。
                        # 旧规则「(换挡次数, 长度) 字典序」会毫不犹豫地选前者 ——
                        # 这就是「总是剐蹭」的直接原因：0 换挡只说明「少揉一把」，
                        # 不说明「这条线离邻车够远」。
                        # 新规则：**先看计划余量够不够**（≥ plan_min_clear），
                        # 都够再比换挡/长度；都不够就取余量大的那个。
                        c_d = self.planner.min_clearance(list(direct.path))
                        c_2 = self.planner.min_clearance(path2)
                        # 两段式的换挡数 = 第一段（开到预备位姿）+ 尾巴那一段。
                        # 尾巴是圆弧时按它自己的换挡数算；是直线时恒为「前进 1 次」。
                        sw2 = (r2.n_switch + (tail_res.n_switch if tail_res else 1))
                        len2 = r2.length + sum(_dist(a, b)
                                               for a, b in zip(tail, tail[1:]))
                        # 除了余量，还要「每一把走得动」：每换挡段至少 MIN_SEG_LEN。
                        # 实测 W4 退化成「2.32 m / 2 换挡」（每段 0.77 m）时，
                        # 跟车器连 shift_gate_arc=0.70 的换挡都判不出来，原地反复
                        # 换挡，最后 1.27 m / 25°、车体只剩 0.04 m。
                        ok_d = (c_d >= self.plan_min_clear
                                and direct.length / max(1, direct.n_switch + 1) >= MIN_SEG_LEN)
                        ok_2 = (c_2 >= self.plan_min_clear
                                and len2 / max(1, sw2 + 1) >= MIN_SEG_LEN)
                        if ok_2 and not ok_d:
                            take = True
                        elif ok_d and not ok_2:
                            take = False
                        elif ok_d and ok_2:
                            # [同学 C 修复] 两个解都「够安全」时，还要再看一眼
                            # **余量差得大不大**。真机 W7 就是在这里吃亏的：
                            # 直连 1 换挡 +0.30 m、两段式 2 换挡 +0.55 m，
                            # 旧规则按「换挡少」选了直连 —— 而离线用真实交接位姿
                            # 跑闭环，直连执行后真实车体间隙只剩 **+0.03 m**，
                            # 两段式那条 0 换挡圆弧是 **+0.22 m**。
                            # 2.5 m 车位的净余量本来只有 0.325 m/侧，0.03 m 等于
                            # 已经在蹭了。所以：余量差距显著（≥0.15 m）时按余量选。
                            if c_2 >= c_d + SAFE_CLEAR_MARGIN:
                                take = True
                            elif c_d >= c_2 + SAFE_CLEAR_MARGIN:
                                take = False
                            else:
                                take = (sw2, len2) < (direct.n_switch, direct.length)
                        else:
                            take = c_2 > c_d
                        if take:
                            res, used_stage, stage_tail = r2, True, tail
                            self.get_logger().info(
                                f'选两段式：余量 {c_2:+.2f} m / {sw2} 换挡 '
                                f'（直连 {c_d:+.2f} m / {direct.n_switch} 换挡）')
                        else:
                            self.get_logger().info(
                                f'选直连：余量 {c_d:+.2f} m / {direct.n_switch} 换挡 '
                                f'（两段式 {c_2:+.2f} m / {sw2} 换挡）')
            if res is None:
                # 起点被占据时的恢复：改用**预备位姿**当规划起点，路径仍是
                # 「从车位口直着倒进去」。
                #
                # 试过把它换成「先倒退 1.5 m 脱困、再重新规划」，实测更差：
                # 自车在那个位姿上**持续被判为占据**，倒退 1.5 m 后依旧占据
                # （日志里连打 55 次「脱困」、始终规划不出结果），只能放弃
                # 车位。所以这里保留旧兜底 —— 它至少能给出可行解。
                if (not direct.success and '占据' in str(direct.reason)
                        and _dist(self.pose, goal) < 6.0):
                    r3 = self.planner.plan(stage_r, goal_r)
                    if r3.success:
                        res = r3
                        self.get_logger().warn(
                            f'起点被占位，改以预备位姿为规划起点'
                            f'（{r3.length:.1f} m，换挡 {r3.n_switch} 次）')
                if res is None:
                    res = direct
        else:
            # 平行车位按三步走，从便宜到贵：
            # 1) 只试 Reeds-Shepp 解析解（不展开栅格）。路侧平行车位的起终点
            #    通常能直接解析连上（实测 0 次扩展、3.9 m 一把倒进去），这条路径
            #    最干净，跟车器不会在换挡点来回抖；
            # 2) 解析解不行就拆成「开到预备位姿」+「从预备位姿揉库」两个短问题
            #    （场内平行车位走这条，总耗时从几十秒降到 3 s 级）；
            # 3) 都失败再退回一次完整搜索。
            saved = self.planner.max_iter
            self.planner.max_iter = 1
            quick = self.planner.plan(self._rear(self.pose), goal_r)
            self.planner.max_iter = saved
            if quick.success:
                res = quick
            if res is None:
                a = self.planner.plan(self._rear(self.pose), stage_r)
                if a.success:
                    b = self.planner.plan(stage_r, goal_r)
                    if b.success:
                        res = PlanResult(
                            # b.path 的第一个点就是预备位姿，与 a.path 的末点重合，
                            # 拼路径时要去掉，否则重复点会让跟车器在换挡点附近来回抖
                            True, list(a.path) + list(b.path)[1:],
                            cost=a.cost + b.cost,
                            expansions=a.expansions + b.expansions,
                            reason=f'两段式 {a.reason}/{b.reason}',
                            used_rs=a.used_rs or b.used_rs,
                            length=a.length + b.length,
                            reverse_length=a.reverse_length + b.reverse_length,
                            n_switch=a.n_switch + b.n_switch)
                        two_stage_par = True
            if res is None:
                res = self.planner.plan(self._rear(self.pose), goal_r)

        dt = time.time() - t0
        self.plan_info = {
            'success': res.success, 'length': res.length,
            'reverse': res.reverse_length, 'switch': res.n_switch,
            'expansions': res.expansions, 'used_rs': res.used_rs,
            'reason': res.reason, 'time': round(dt, 2),
            'two_stage': used_stage or two_stage_par,
            'kind': self.target.kind,
        }
        if not res.success:
            self.get_logger().error(
                f'车位 {self.target.id} 规划失败（{res.reason}），加入黑名单')
            # [B3 修复] 同一车位**连续**规划失败要能收手。
            # 强制指定车位时（`goal_stall:=B1`）黑名单根本不生效 —— 选位逻辑
            # 会把同一个位再选一次，于是「选位 → 规划失败 → 拉黑 → 又选中」
            # 每秒一轮死循环，直到批次超时白跑 15 分钟（真机 B1 本轮）：
            #   ERROR 车位 B1 规划失败（goal 位姿被占据（车体与障碍重叠））
            #   INFO  选定车位 B1（... free_ratio=1.00）        ← 立刻又选中
            # 连续 3 次就按 INCOMPLETE 收场，把时间留给下一个车位。
            self._plan_fail_n += 1
            if self._plan_fail_n >= PLAN_FAIL_MAX:
                fid = self.target.id
                self.blacklist.add(fid)
                self.get_logger().error(
                    f'车位 {fid} 连续 {self._plan_fail_n} 次规划失败，放弃本次泊车')
                self._plan_fail_n = 0
                self._finish(now)
                return
            self.blacklist.add(self.target.id)
            self.target = None
            self.state = 'SELECT_SPOT'
            self.retry_at = now + 0.5
            return
        self._plan_fail_n = 0

        path = list(res.path)
        if used_stage and stage_tail is not None:
            path = path + list(stage_tail)
        # [同学 C 修复] 无论直连还是两段式，下发前统一去重（见 `dedup_path`）。
        path = dedup_path(path)
        # 统计口径：一律按最终**下发**的 path 重算。两段式会再补一条「直线倒进
        # 车位」，若沿用 r2 的统计值，评测表里就会出现「日志里一路 档位=R、
        # 但 reverse=0.00」这种自相矛盾（真机两次运行都出现了）。
        seg_pairs = list(zip(path, path[1:]))
        res.length = sum(_dist(a, b) for a, b in seg_pairs)
        res.reverse_length = sum(_dist(a, b) for a, b in seg_pairs if b[3] < 0)
        res.n_switch = sum(1 for a, b in seg_pairs if a[3] != b[3])
        self.plan_info.update({'length': res.length,
                               'reverse': res.reverse_length,
                               'switch': res.n_switch})
        self.path = path
        self.follower.set_path(path)
        self.state = 'FOLLOW'
        self._follow_start = now
        total = sum(_dist(a, b) for a, b in zip(path, path[1:]))
        # 车体余量复核：规划器的碰撞判据是二值的，这里把「离障碍还剩多少」
        # 打出来。小于 0.15 m 说明这条路径只要跟踪误差稍大就会蹭上邻车。
        clr = self.planner.min_clearance(path)
        self.plan_info['clearance'] = round(clr, 3)
        # 记录本条路径承诺的最小余量：倒车护栏用它做「相对」门限（见 _side_guard_ok）
        self._plan_clear = clr
        if clr < 0.15:
            self.get_logger().warn(
                f'入位路径车体余量仅 {clr:+.2f} m（<0.15 m），跟踪时会贴近邻车')
        # [同学 C 本轮新增] 拒绝「明显离谱」的解法。
        # 真机 W5（2026-10-27 15:57）那次就是：交接位姿 0.20 m / 3°，本来应该是
        # 12.27 m / 0 换挡的那条弧，结果规划出 **26.46 m / 3 换挡 / 扩展 40 次 /
        # 余量 +0.20 m** —— 因为起步位姿旁边刚好停了一台干扰车（侧面间隙 0.13 m），
        # 起点本身就压在栅格里，RS 解析解全部失效，只能疯狂展开栅格。
        # 这种解一旦执行就是「在车位口来回揉 3 次、磨掉 126.8 m」，比不执行更糟。
        # 处理：换挡 ≥3 次就判为「起步位姿被临时障碍污染」，停 4 s 重试（障碍通常
        # 会自己开走）；重试 3 次仍离谱才接受，避免永久卡死。
        # [本轮修复 · 建议 2] 「车体余量不够」也进「拒收 → 重搜」分支，
        # 而且第一次拒收时把**搜索预算翻倍**再搜一遍（扩大搜索 = 多揉一把的机会），
        # 而不是原样重搜。
        # 真机证据：P1 连续三轮都是「换挡 3 次 / 12.8 m / 余量 +0.05 m」，
        # 旧判据只看换挡次数，重试 3 次后照样执行 → 终点偏 3.03 m、间隙 0.00 m。
        too_tight = clr < MIN_EXEC_CLEAR
        if (res.n_switch >= 3 or too_tight) and self._plan_retry < 3:
            self._plan_retry += 1
            if self._plan_retry == 1:
                base = int(getattr(self, '_base_max_iter', 6000))
                self.planner.max_iter = min(24000, max(6000, base) * 2)
                self.get_logger().warn(
                    f'规划搜索预算放大到 {self.planner.max_iter} 次扩展')
            self._plan_wait_until = now + 4.0
            self.state = 'PLAN'
            self.stop()
            why = (f'车体余量 {clr:+.2f} m < 执行门限 {MIN_EXEC_CLEAR:.2f} m'
                   if too_tight
                   else f'换挡 {res.n_switch} 次 / {total:.1f} m')
            self.get_logger().warn(
                f'规划解法不可执行（{why}），等 4 s 重搜'
                f'（{self._plan_retry}/3）')
            return
        if self._plan_retry:
            # 恢复基准搜索预算，避免后续车位一直用放大后的预算（慢）
            self.planner.max_iter = int(getattr(self, '_base_max_iter', 6000))
            if too_tight:
                self.get_logger().warn(
                    f'重搜 3 次仍只有车体余量 {clr:+.2f} m 的解，'
                    f'按当前最优解执行（执行中由侧向护栏兜底）')
        self._plan_retry = 0
        self.get_logger().info(
            f'泊车规划成功：{total:.2f} m（倒车 {res.reverse_length:.2f} m，'
            f'换挡 {res.n_switch} 次，扩展 {res.expansions} 次，'
            f'车体余量 {clr:+.2f} m，'
            f'RS={res.used_rs}，两段式={used_stage}，耗时 {dt*1000:.0f} ms）')
        self.publish_path()

    @staticmethod
    def _rear(pose):
        """中心位姿 → 后轴位姿（§6.29，进规划器前的统一换算）。yaw 不变。"""
        c, s = math.cos(pose[2]), math.sin(pose[2])
        return (pose[0] - c * REAR_OFF, pose[1] - s * REAR_OFF, pose[2])

    def _straight_ok(self, p0, p1):
        n = max(2, int(_dist(p0, p1) / 0.10))
        for i in range(n + 1):
            t = i / n
            if not self.planner.free(p0[0] + (p1[0] - p0[0]) * t,
                                     p0[1] + (p1[1] - p0[1]) * t, p0[2]):
                return False
        return True

    # ------------------------------------------------------------------ 执行
    def _yaw_rate(self, now):
        """实测偏航率（rad/s）。**优先用 IMU**，没有 IMU 才退回位姿差分。

        [同学 C 本轮] 位姿差分这条路在本项目里基本是废的：`/vehicle_pose` 由
        `map→odom`（slam_toolbox，只在扫描匹配出校正时才跳一下）复合而成，
        20 Hz 差分出来**绝大多数帧就是 0.00**（真机日志 `ω实测=+0.00` 连续几十行）。
        拿 0 去喂阻尼等于没有阻尼，而航向环又确实因为「一整个 SLAM 校正周期的
        延迟 + 0.25 s 转向滞后」而过冲（φ 在 −12°~+22° 摆）。
        IMU（100 Hz）给的是真实车体角速度，延迟只有一个控制周期，才能真正当微分项用。

        注意符号：IMU 的 z 轴角速度与 `wrap(Δyaw)` 同号（同一套 base_link 右手系），
        不需要取反；倒车时也不变号 —— 跟踪器里的 `k_rate` 项本来就是按「实测 ω
        与期望 ω 的差（带符号）」推的（见 controller.update 的 yaw_rate 分支）。
        """
        if self.imu_wz is not None and (now - self.imu_stamp) < 0.30:
            return self.imu_wz
        # [同学 C 本轮修复 —— 这条是上一轮引入的严重副作用]
        # 原来这里会退回「位姿差分」。但 `ω实测` 在真机日志里**一直是 +0.00**：
        # `/vehicle_pose` 的航向来自 slam_toolbox 的 map→odom，平时是阶梯式跳变，
        # 逐帧差分绝大多数为 0。而 `0.0` **不是 None**，跟踪器里
        #     curvature -= k_rate * (yaw_rate - w_ref) / vv
        # 就变成 `curvature += k_rate * curvature` —— **k_rate=0.8 时等于把曲率
        # 指令放大 1.8 倍**。真机 W5 日志里 κ 从第一拍就钉在 +0.202 的限幅上、
        # 横向偏差 4 m 内从 0.10 涨到 1.01 m，就是这 1.8 倍造成的：
        # 车实际按 R≈5.2 m 转，而规划是 5.74 m，整条弧偏掉、最后卡在离车位 5 m 处。
        # 修法：拿不到可信偏航率就**明确返回 None**，跟踪器会跳过阻尼项（回到
        # 上一轮已验证过的那套纯比例反馈），绝不能让"没有信号"被当成"信号为零"。
        if self.imu_miss is None:
            self.imu_miss = now
            self.get_logger().warn(
                f'还没收到 {self.get_parameter("imu_topic").value} 的偏航率，'
                f'入位段的偏航率阻尼自动关闭（不会退化成放大增益）')
        return None

    # ------------------------------------------------------------------ ALIGN
    def _handoff(self, now):
        """「全局路线 → 入位」的交接点：默认先跑一次 ALIGN 对齐机动，再入位。

        为什么要先对齐（方案 A）
        ------------------------
        垂直车位的预备位姿与过道**正交**，路线结束时自车距它 4.6~5.6 m、
        航向差 80~94°。直接在这个位姿上规划入位，Hybrid A* 只能搜出 9~14 m、
        换挡 1~3 次的揉库解；而 2.5 m 车位的净余量只有 0.325 m，多换挡路径的
        跟踪误差（实测横向 0.9~1.2 m）直接把这点余量吃穿 —— 这就是「总是剐蹭」
        和「多把揉库」的共同来源。

        先把车**精确摆到预备位姿**（±0.30 m / ±12°），入位就只剩一条直线倒车：
        离线全量回归（`scripts/verify_align_sim.py`，11 个可用垂直车位、
        交接位姿回退 0.25 m + 横偏 0.25 m）由此得到 **入位全部 0 换挡、
        0 剐蹭（执行轨迹真实车体间隙 +0.17~+0.97 m）、终点 0.14~0.15 m**。

        关闭方式：launch 参数 `use_align:=false`（退回原来的直接入位）。
        """
        # [同学 C 修复] ALIGN 只在「预备位姿本来就在附近」时才跑。
        #
        # 原意是「先摆正到预备位姿，入位就只剩直线倒车」，但场内垂直车位的预备
        # 位姿**物理上到不了**：过道宽 12.5 m（半宽 6.25），而最小转弯半径 5.74，
        # 从过道中线做 90° 转弯会扫到 x=−2.26、车身压过绿化岛边缘（−1.75）——
        # 过道里摆不出「车头朝过道、车身贴车位轴」这个位姿；它只能从车位里倒车
        # 到达。于是 ALIGN 被迫执行一条 10.7 m 的 S 形机动（先前进拉开、再倒车），
        # 分段执行时每段对「距预备位姿」的净收益不到 1 m，真机实测用满 10 段也到
        # 不了（W4 2.03 m、W5 0.64 m 都是这么来的）。
        #
        # 而交接点现在已经精确落在**车道节点**上（`remaining < 0.25 m`），离线全量
        # 数据显示：从这个点直接规划入位是 **0 换挡、4.5~9.2 m 的纯倒车**，
        # 车体间隙 +0.34~+0.97 m、终点 0.14~0.15 m —— 也就是「一把倒进去」，
        # 正是 ALIGN 想要的结果，却不需要那段 S 形机动。
        #
        # 所以判据改成：预备位姿本来就在 `align_max_dist` 以内（路侧平行位只有
        # 1.6 m，ALIGN 一段就收敛）才跑 ALIGN；否则直接从车道节点入位。
        d_stage = _dist(self.pose, self.target.staging_pose)
        max_d = float(self.get_parameter('align_max_dist').value)
        # [同学 C 本轮] 已经站在预备位姿的容差内就**不要再对齐**。
        # 真机 W7 的交接位姿是「0.4 m / 9°」，本来就已经够准；ALIGN 却先往前开
        # 2.5 m 再倒回来（日志：`对齐到预备位姿：全长 5.7 m（倒车 2.8 m，换挡 1 次）`），
        # 白耗 30 s，还把位姿从 0.43 m 推到 2.4 m 以外 —— 于是 `_plan` 里
        # 「锚定预备位姿」的判据不成立，又退回从偏位姿规划（计划间隙 +0.18）。
        #
        # [同学 C §6.31] 判据从「欧氏 1.2 m」改成**沿 / 横分解、各 0.30 m**。
        # 真机 W7 这一轮（282.5 s / INCOMPLETE / 间隙 0.00 m）是反例：航向 0°、
        # 沿过道差 1.13 m，欧氏距离仍 < 1.2 m → 跳过 ALIGN + 锚定到预备位姿 →
        # 跟踪从 e=-0.69 m 起步，倒车弧侧向扫过邻车（侧面间隙 5.31→0.28→0.01 m）。
        # 分解后：横向超差 → 弧的几何真的偏了，绝不能再锚定；纵向超差 → 补一段
        # 直道就能回到弧的起点，交给 ALIGN（航向差 0° 时它就是一条直线，不会
        # 出现「先开出去 2.5 m 再倒回来」那种 s 形机动）。
        _st_along, _st_lat = stage_offset(self.pose, self.target.staging_pose)
        already = (abs(_st_along) <= STAGE_XY_TOL
                   and abs(_st_lat) <= STAGE_XY_TOL
                   and abs(wrap(self.pose[2] - self.target.staging_pose[2]))
                   < math.radians(12.0))
        if already:
            self.get_logger().info(
                f'已在预备位姿容差内（{d_stage:.2f} m = 沿 {_st_along:+.2f} / '
                f'横 {_st_lat:+.2f} m，'
                f'{math.degrees(abs(wrap(self.pose[2] - self.target.staging_pose[2]))):.0f}°），'
                f'跳过 ALIGN，直接按预备位姿锚定的圆弧入位')
            self._plan(now)
            return
        self.get_logger().info(
            f'未到预备位姿容差（{d_stage:.2f} m = 沿 {_st_along:+.2f} / '
            f'横 {_st_lat:+.2f} m，门限 {STAGE_XY_TOL:.2f} m），'
            f'不锚定，先对齐')
        want_align = (bool(self.get_parameter('use_align').value)
                      and os.environ.get('USE_ALIGN', '1')
                      not in ('0', 'false', 'False'))
        # [本轮修复 · 平行位] **平行位一律先摆到预备位姿，不再「从车道节点直接入位」。**
        # 真机证据（2026-10-28 平行位回归）：
        #   P1 交接时距预备位姿 7.26 m（沿 +7.16 / 横 -1.20 m）/ 88°，
        #   P3 是 3.52 m / 84°；两者都因为 d_stage > align_max_dist(2.5) 走了
        #   「跳过 ALIGN，从车道节点入位」，Hybrid A* 只能搜出
        #   「换挡 3 次 / 12.8 m / 车体余量 +0.05 m」的揉库解 ——
        #   P1 执行后终点偏差 3.03 m / 17.2°，P1、P3 车体最小间隙都是 0.00 m。
        # 根本原因：垂直位的预备位姿与过道正交，但它的「车道节点 → 车位」本来
        # 就是一条已验证的弧；而平行泊车的入位动作强依赖「车身与车位平行、停在
        # 车位前方」，车头朝着过道方向（差 84~88°）时任何一条弧都会扫到两侧邻车。
        # 所以平行位即使差得远也必须先做 ALIGN；`_begin_align` 内部规划失败时
        # 会自己退回 `_plan`，不会卡死。
        parallel = (self.target.kind == 'parallel')
        # [本轮修复 · 补丁] 平行位「强制 ALIGN」必须**限定距离**。
        # 第一版没加距离条件，结果路线中途被挡时 `_route_tick` 提前交接，
        # 自车还在 60 m 外就触发了「平行位先对齐机动」——ALIGN 要规划 60 m，
        # 真机 B3 a3 就是这么废掉的（日志 `距预备位姿 60.1 m ... 先摆正再入位`）。
        # ALIGN 的语义是「最后几米的摆正」，超过一个车身段就没有意义，
        # 交给 `_plan` 的「起点被占据/直接入位」逻辑处理更合适。
        if parallel and d_stage > PARALLEL_ALIGN_MAX_DIST:
            self.get_logger().warn(
                f'平行位距预备位姿仍有 {d_stage:.1f} m（> '
                f'{PARALLEL_ALIGN_MAX_DIST:.0f} m），不做对齐机动，直接入位规划')
            self._plan(now)
            return
        if want_align and (parallel or d_stage <= max_d):
            if parallel and d_stage > max_d:
                self.get_logger().info(
                    f'平行位先对齐机动：距预备位姿 {d_stage:.1f} m'
                    f'（沿 {_st_along:+.2f} / 横 {_st_lat:+.2f} m，'
                    f'航向差 '
                    f'{math.degrees(abs(wrap(self.pose[2] - self.target.staging_pose[2]))):.0f}°），'
                    f'超出常规门限 {max_d:.1f} m 也先摆正再入位')
            self._begin_align(now)
        else:
            if want_align:
                self.get_logger().info(
                    f'预备位姿还在 {d_stage:.1f} m 外（> {max_d:.1f} m），'
                    f'跳过 ALIGN，直接从车道节点入位（0 换挡纯倒车）')
            self._plan(now)

    def _escape(self, now, dist=1.5):
        """起点被占据时的脱困机动：沿车头反方向直线退一小段，退完重新规划。

        为什么必须是「从自车当前位姿出发」：任何起点不是自车实际位置的路径，
        跟车器一开始就带着等于偏移量的横向偏差，倒车段根本收不回来（实测
        `横向偏差=4.32 m 档位=R`，4.5 m 的倒车结束时偏差仍是 4.3 m）。
        倒退 1.5 m 是几何上一定可跟随的机动，也是揉库时脱离「车身压线」
        最自然的动作。
        """
        # [同学 C 本轮 §6.29] 入位段路径一律**后轴语义**：手工构造的脱困直线
        # 也从后轴位姿出发（`follower_park` 的 ref_ahead=0 会把反馈位姿换算到
        # 后轴，起点不换算的话第一拍就带 1.35 m 的纵向假偏差）。
        x, y, th = self._rear(self.pose)
        n = max(3, int(dist / 0.25))
        path = []
        for i in range(n + 1):
            u = i / float(n)
            path.append((x - math.cos(th) * dist * u,
                         y - math.sin(th) * dist * u, th, -1, 0.0))
        self.path = path
        self.follower = self.follower_park
        self.follower.set_path(path)
        self._follow_start = now
        self._escape_pending = True
        self.state = 'FOLLOW'
        self.publish_path()
        self.get_logger().warn(
            f'起点被占据，先倒退 {dist:.1f} m 脱困，随后重新规划'
            f'（剩 {self._escape_left} 次）')

    def _retry_approach(self, now, reason):
        """ALIGN 未把车摆到预备位姿时的**统一出口**：退回路线重接入，而不是带偏入位。

        [同学 C §6.36] 真机 B3 的失败链就缺这一步：交接横向差 1.17 m → ALIGN 擦着
        `parked_b2`（间隙 0.01 m）→ 45 s 超时 → 旧代码无条件「按当前位置直接入位」
        → 带 1 m 横向偏差起弧 → 249 s 内 1644 次护栏才揉进车位，最小间隙 0.00 m。

        现在的规则：横向残差 > `ALIGN_HANDOFF_LAT_MAX` 时**不许**入位，先回路线图
        从当前位姿重新接到预备位姿（最后十几米会走 `_blend_to_staging` 的 S 形接入），
        最多 `ALIGN_REAPPROACH_MAX` 次；用尽才带偏入位，并明确告警留证。

        为什么斜向残差只看「横向」：纵向差 1 m 只是「起点早/晚一米」，入位那条直线
        会把 1 m 吃进倒车长度里（离线实测纵向 1.13 m 时入位仍 0 换挡）；而横向差
        1 m 意味着整条弧的几何都偏了，只能靠贴边揉库找回来。
        """
        st = self.target.staging_pose
        _, lat = stage_offset(self.pose, st)
        self.stop()
        self.planner = None
        if abs(lat) > ALIGN_HANDOFF_LAT_MAX and self._reapproach_left > 0:
            self._reapproach_left -= 1
            route_r = self.route_graph.plan(self.pose, st,
                                            start_heading=self.pose[2])
            if route_r:
                route_r = self._extend_to_staging(route_r, st)
                self._begin_route(route_r, now)
                self.get_logger().warn(
                    f'{reason}，横向残差 {lat:+.2f} m > '
                    f'{ALIGN_HANDOFF_LAT_MAX:.2f} m，退回路线重新接入预备位姿'
                    f'（剩 {self._reapproach_left} 次）')
                return
            self.get_logger().warn(f'{reason}，重接入路线失败，按当前位姿入位')
        elif abs(lat) > ALIGN_HANDOFF_LAT_MAX:
            self.get_logger().warn(
                f'{reason}，横向残差 {lat:+.2f} m 且重接入次数用尽，'
                f'按当前位姿入位（可能贴边揉库）')
        else:
            self.get_logger().info(
                f'{reason}，横向残差 {lat:+.2f} m 在 '
                f'{ALIGN_HANDOFF_LAT_MAX:.2f} m 容差内，按当前位姿入位')
        self._plan(now)

    def _begin_align(self, now):
        """ALIGN：用 Hybrid A* 从当前位姿精确对齐到预备位姿。

        为什么必须是一个独立状态，而不是塞进路线跟踪
        --------------------------------------------
        垂直车位的预备位姿与过道是**正交**的（车头朝车道、车尾朝车位），
        路线结束时自车距它 4.6 m、航向差 91~94°。这个 90° 弯靠「沿车道
        继续开」到不了：4.7 m 内转 91° 需要 κ≈0.34 rad/m，而车的极限只有
        0.20（实测曲率上限），跟车器会直接卡住并报「路线超时」。

        所以对齐交给带运动学约束的 Hybrid A* 做，它自己能切角、必要时换挡。
        对齐完成后误差只到 0.6 m / 12° 以内，再做「预备位姿→车位」的 4.5 m
        直线倒车 —— 前一阶段的终点误差成为后一阶段的起点误差，不会被放大。
        （原来直接在 4.6 m / 92° 的位姿上手入位，Hybrid A* 只能搜出 12~14 m、
        换挡 1~2 次的揉库解，终点误差被放大到 1.8~2.9 m。）
        """
        self._build_local_planner()
        out = self.planner.plan(self._rear(self.pose),
                                self._rear(self.target.staging_pose))
        if not out.success:
            self.get_logger().warn(
                f'对齐规划失败（{out.reason}），退回「直接入位」')
            self._plan(now)
            return
        # 保留 planner：分段滚动执行要在同一个窗口里反复重规划。
        # 只在真正交接给 _plan 时才清空（那时自车已换位姿，必须重建窗口）。
        self.path = self._trim_path(out.path, ALIGN_HORIZON)
        self.follower = self.follower_park
        self.follower.set_path(self.path)
        self._align_len = out.length
        self._align_retry = 0
        self._align_n = 0
        self._align_anchor = (self.pose[0], self.pose[1])
        self._align_best = _dist(self.pose, self.target.staging_pose)
        self._align_prev_d = self._align_best
        self._align_stall = 0
        self._align_guard_n = 0          # [§6.36] 每次进 ALIGN 重新数护栏
        self._follow_start = now
        self.state = 'ALIGN'
        self.publish_path()
        self.get_logger().info(
            f'对齐到预备位姿：全长 {out.length:.1f} m（倒车 {out.reverse_length:.1f} m，'
            f'换挡 {out.n_switch} 次，扩展 {out.expansions} 次）；'
            f'分段滚动执行，首段 {self._path_len(self.path):.1f} m')

    @staticmethod
    def _path_len(path):
        return sum(math.hypot(path[i][0] - path[i - 1][0], path[i][1] - path[i - 1][1])
                   for i in range(1, len(path)))

    def _trim_path(self, path, horizon):
        """把路径截到约 `horizon` 米弧长，用于分段滚动执行。

        [同学 C 修复] **截断点不许早于第一个换挡点**，否则多点机动会原地打转。
        实测真机 W5：

            对齐第 1 段：规划 9.0 m，执行 2.5 m   距预备位姿 5.47 → 5.92 → 6.55
            对齐第 2 段：规划 7.9 m，执行 2.6 m   6.35 → 5.56
            对齐第 3 段：规划 7.1 m，执行 2.5 m   5.34 → 5.78

        这条机动是「先前进 ~2.5 m 拉开距离，再倒车 7.1 m 进来」，而分段弧长也正好
        是 2.5 m —— 于是每一段都只走完「拉开」那一半就重规划，重规划又是「拉开」，
        车在 5.3~6.5 m 之间来回，永远到不了预备位姿。把第一个换挡点包进本段，
        重规划就发生在「已拉开、该倒车」的位姿上，天然不会重复。

        原注释保留：一次性执行 9.4 m / 1 次换挡的紧凑 RS 机动时，跟车器累积了
        3.7 m 偏差（`剩余` 降到 1 m 而 `距预备位姿` 始终 4 m），所以仍然要分段。
        """
        if len(path) < 3:
            return path
        # 到第一个换挡点的弧长
        arc = 0.0
        g0 = path[0][3]
        seg_start = 0.0          # 当前这一「把」（同一档位）的起点弧长
        cap = horizon * 3.0      # 兜底：本段不超过 3 倍目标弧长
        extended = False
        for i in range(1, len(path)):
            arc += math.hypot(path[i][0] - path[i - 1][0],
                              path[i][1] - path[i - 1][1])
            if path[i][3] == g0:
                continue
            leg_len = arc - seg_start
            g0 = path[i][3]
            seg_start = arc
            if leg_len >= MIN_SEG_LEN:
                # 这一把走得动：换挡点包进本段；若它本来就在 horizon 之外则照旧切
                if extended or arc < horizon:
                    horizon = arc + 0.5
                break
            # [同学 C 修复] 这一把太短（< MIN_SEG_LEN），把下一把也包进来。
            # 场内垂直位的对齐机动是「前进 1.1 m → 倒车 6 m → 再前进」：
            # 只包第一把时每段都只走完那 1.1 m 就被切掉，重规划出来的还是同一
            # 小步「拉开」，总长一直停在 7.9~8.4 m 不收敛 —— 真机 W4 上
            # ALIGN 连跑 6 段 / 45 s 后超时放弃。包进第二把，每段才能真正往前走。
            horizon = min(arc + 0.5, cap)
            extended = True
            if arc >= cap:
                break
        out = [path[0]]
        acc = 0.0
        for p in path[1:]:
            acc += math.hypot(p[0] - out[-1][0], p[1] - out[-1][1])
            out.append(p)
            if acc >= horizon:
                break
        return out if len(out) >= 2 else path

    def _replan_align(self, now):
        """当前段走完后，从**自车实际位姿**重新规划下一段。

        另外带一个「平台期」判据：分段滚动到最后通常会停在「位置基本到了、
        航向差几度」的平台，这时再规划只是原地打转（离线实测有车位白白用掉
        10 段、35 s）。所以连续两段改善都 < 0.02 m 且距离已经 ≤0.40 m 时，
        就认定「够用了」，直接交接给入位 —— 入位那条直线本来就会吃下这点残差。
        """
        st = self.target.staging_pose
        d_now = _dist(self.pose, st)
        if d_now <= 0.40 and (self._align_prev_d - d_now) < 0.02:
            self._align_stall += 1
        else:
            self._align_stall = 0
        self._align_prev_d = d_now
        if self._align_stall >= 2:
            self.get_logger().info(
                f'对齐进入平台期（{d_now:.2f} m，连续 {self._align_stall} 段无改进）')
            self._retry_approach(now, '对齐进入平台期')
            return
        if self.planner is None:
            self._build_local_planner()
        out = self.planner.plan(self._rear(self.pose), self._rear(st))
        if not out.success:
            self.get_logger().warn(f'对齐重规划失败（{out.reason}）')
            self.planner = None
            self._retry_approach(now, '对齐重规划失败')
            return
        self._align_n += 1
        self._align_anchor = (self.pose[0], self.pose[1])
        self._align_guard_n = 0          # [§6.36] 新一段重新数护栏
        self.path = self._trim_path(out.path, ALIGN_HORIZON)
        self.follower = self.follower_park
        self.follower.set_path(self.path)
        self.publish_path()
        self.get_logger().info(
            f'对齐第 {self._align_n} 段：规划 {out.length:.1f} m，'
            f'执行 {self._path_len(self.path):.1f} m'
            f'（换挡 {out.n_switch} 次，扩展 {out.expansions} 次）')

    def _align_tick(self, now):
        if now - self._follow_start > ALIGN_TIMEOUT:
            self._retry_approach(now, f'对齐超时（{ALIGN_TIMEOUT:.0f}s）')
            return
        if self.risk == 'danger':
            self.stop()
            return

        st = self.target.staging_pose
        d_stage = _dist(self.pose, st)
        yaw_stage = abs(wrap(self.pose[2] - st[2]))

        # ---- 发散保护：越走越远就停手 ----
        # 历史日志出现过「对齐结束时离预备位姿 4.3 m，比交接时还远」。分段滚动
        # 本身会把误差吃掉，但一旦某一段的规划/跟踪方向不对，就会一段比一段远。
        # 这里记住历史最好成绩：当前距离比它差 0.5 m 以上就判定发散，立刻交给
        # `_plan` 从当场位姿入位（最坏也就等于「没有 ALIGN」）。
        if d_stage < self._align_best:
            self._align_best = d_stage
        elif (self._align_n >= ALIGN_BEST_MIN_SEG
              and d_stage > self._align_best + ALIGN_BEST_MARGIN):
            self.get_logger().warn(
                f'对齐发散（当前 {d_stage:.2f} m，最好 {self._align_best:.2f} m）')
            self._retry_approach(now, '对齐发散')
            return

        # 到点判据按**真实位置**（对准预备位姿），不看路径末点 ——
        # 分段执行时路径末点只是本段的终点，不是预备位姿。
        if d_stage <= ALIGN_XY_TOL and yaw_stage <= ALIGN_YAW_TOL:
            self.get_logger().info(
                f'对齐完成：距预备位姿={d_stage:.2f} m '
                f'航向差={math.degrees(yaw_stage):.1f}° '
                f'自车=({self.pose[0]:.2f},{self.pose[1]:.2f}) '
                f'共分 {self._align_n + 1} 段执行')
            self.stop()
            # 自车已换位姿，交给 _plan 时重建窗口
            self.planner = None
            self._plan(now)
            return

        scale = 0.45 if self.risk == 'warning' else 1.0
        cmd = self.follower.update(self.pose, 0.05, speed_scale=scale,
                                   yaw_rate=self._yaw_rate(now))

        # 本段走完 / 卡住 / **横向跑偏过大** → 从当前实际位姿重规划下一段
        # （横向这条是新增的：等一段 2.5 m 走完再重规划，跑偏已经累积到 1 m 以上，
        #   而 ALIGN 的下一段起点就是入位规划的起点，误差会一路带进车位）
        if cmd.done or cmd.stuck or abs(cmd.lateral_error) > ALIGN_LAT_ABORT:
            if self._align_n >= ALIGN_REPLAN_MAX:
                self.get_logger().warn(
                    f'对齐分段用尽（{ALIGN_REPLAN_MAX} 段，'
                    f'距预备位姿 {d_stage:.2f} m）')
                self._retry_approach(now, '对齐分段用尽')
                return
            self._replan_align(now)
            return

        tw = Twist()
        tw.linear.x = float(cmd.v)
        tw.angular.z = float(cmd.omega)
        # 入位段也让人：车体扫过站立行人时，0.2 m 的激光护栏已经太晚了。
        # 半径取小值，只在人贴到车身旁边时才停，避免行人路过车位口就打断揉库。
        if self._ped_in_front(float(self.get_parameter('ped_yield_park').value)):
            self.stop()
            if now - self._ped_log > 1.5:
                self._ped_log = now
                self.get_logger().info('入位让行：行人贴近车体，暂停')
            return
        if not self._side_guard_ok(now, gear=cmd.gear, remaining=cmd.remaining):
            # [同学 C §6.36] 累计护栏次数（**不**按帧清零）：ALIGN 期间一直擦着邻车
            # 走，说明这条段本身不可执行，不能硬推到 45 s 超时。真机 B3 在 ALIGN 的
            # 45 s 里触发了 87 次（约 2 次/s），车体到 `parked_b2` 只剩 0.01 m；
            # 旧逻辑对此毫无反馈，超时后还把这份偏差带进了入位弧。
            self._align_guard_n += 1
            if self._align_guard_n >= ALIGN_GUARD_ABORT:
                self.get_logger().warn(
                    f'对齐段累计贴合护栏 {self._align_guard_n} 次，'
                    f'判本段不可执行（距预备位姿 {d_stage:.2f} m）')
                self._align_guard_n = 0
                self._retry_approach(now, '对齐段持续贴合护栏')
            return
        self.pub_cmd.publish(tw)
        if now - self._last_cmd_log > 2.0:
            self._last_cmd_log = now
            self.get_logger().info(
                f'对齐中 v={cmd.v:.2f} m/s ω={cmd.omega:+.2f} rad/s '
                f'本段剩={cmd.remaining:.2f} m 距预备位姿={d_stage:.2f} m '
                f'档位={"D" if cmd.gear > 0 else "R"}')

    def _follow(self, now):
        if now - self._follow_start > 120.0:
            self.get_logger().error('跟踪超时（120 s），放弃当前车位')
            self.blacklist.add(self.target.id)
            self.target = None
            self.state = 'SELECT_SPOT'
            self.retry_at = now + 0.5
            self.stop()
            return

        if self.risk == 'danger':
            if self.danger_since is None:
                self.danger_since = now
                self.get_logger().warn('路径被遮挡，停车等待')
            held = now - self.danger_since
            self.stop()
            # [同学 C] 只在「等待超时」时重新规划（原来每 6 s 就重规划一次）。
            # 行人在旁边徘徊时，重规划只能给出一条「往前开」的新路，于是表现为
            # 「车被行人一路逼着往前走、最后停不进指定车位」。
            # 正确做法是原地等：行人让开（新的走廊让行规则会主动避让）后 danger
            # 自然解除，下一帧直接用原路径继续，不需要任何重规划。
            if held > 30.0:
                self.get_logger().warn(f'遮挡持续 {held:.1f}s，重新规划')
                self.danger_since = None
                self._plan(now)
            return
        self.danger_since = None
        scale = 0.45 if self.risk == 'warning' else 1.0

        cmd = self.follower.update(self.pose, 0.05, speed_scale=scale,
                                   yaw_rate=self._yaw_rate(now))
        # [同学 C 本轮] 记下实测偏航率，供「执行器欠转系数」标定：
        # 日志里 `ω实测/ω指令` 的稳态值就是 steer_gain 应该取 1/该比值。
        self._yaw_rate_meas = self._yaw_rate(now)

        # 进度看门狗：跟踪器报「路径剩余距离连续 5 s 不减少」。
        # 这正是「车还在动、但路径进度不动」的死锁——换挡点差一点点够不到就
        # 永远不换挡，于是一直朝一个方向开，日志里 remaining 冻在某个值、
        # 看起来就是车「停驻」在那里。这里改成重新规划。
        if cmd.stuck:
            # [B3 修复] 先问一句「挡路的是不是会动的干扰车」：是 → 冻结等待。
            # 「等别人让路」和「自己几何卡死」不该烧同一个预算（真机 B3 就是
            # 在蓝车倒退完成前 5 s 耗尽 5 次微调预算放弃的）。
            if self._agent_hold(now, cmd):
                return
            if cmd.remaining < 0.05:
                # 「路径索引走完、车却没到终点」：这不是真停滞，是索引跑得比车快。
                # 用独立计数重规划一小段，把最后那半米补上。
                self._end_retry += 1
                if self._end_retry <= END_RETRY_MAX:
                    gx, gy, gth = self.target.goal_pose
                    err = math.hypot(gx - self.pose[0], gy - self.pose[1])
                    self.get_logger().warn(
                        f'路径已走完但未到位（距目标 {err:.2f} m），'
                        f'第 {self._end_retry} 次按当前位姿补规划')
                    self.stop()
                    self._plan(now)
                else:
                    self.get_logger().warn(
                        f'补规划次数用尽（{END_RETRY_MAX} 次），按当前位姿结束')
                    self._finish(now)
                return
            # [B3 修复] 停滞处理改**按时间节流**，不再按拍计数。
            # 旧写法 `_stuck_hits >= 2` 的假设是「stuck 是个偶发事件」，但
            # `cmd.stuck` 是**持续状态**：一旦 `_stall_time` 超过 5 s 就每拍都为
            # True，于是 2 拍（0.1 s）就吃掉一次微调预算 —— 只要中间有一次
            # `_plan` 没成功（规划被拒收/搜索失败时不会 `set_path`，
            # `_stall_time` 也不会清零），5 次预算会在 0.6 s 内雪崩烧光。
            # 真机 B3 本轮：1793294798「泊车规划成功」→ 1793294806
            # 「进度停滞且微调次数用尽」，中间只有 8 s、6 条「倒车被后方障碍
            # 挡住」，预算就已经用完，车其实只差一点余量就能倒进去。
            # 现在两次停滞处理之间至少隔 STUCK_RETRY_GAP 秒，预算按真实
            # 「尝试次数」消耗，而不是按控制周期消耗。
            if now - self._stuck_t < STUCK_RETRY_GAP:
                self.stop()
                return
            self._stuck_t = now
            self._stuck_hits = 0
            self._dock_retry += 1
            if self._dock_retry <= DOCK_RETRY_MAX:
                self.get_logger().warn(
                    f'路径进度停滞（剩余 {cmd.remaining:.2f} m，'
                    f'横向 {cmd.lateral_error:.2f} m，'
                    f'档位={"D" if cmd.gear > 0 else "R"}），重新规划')
                self.stop()
                self._plan(now)
            else:
                self.get_logger().warn(
                    f'进度停滞且微调次数用尽（剩余 {cmd.remaining:.2f} m），'
                    f'按当前位姿结束')
                self._finish(now)
            return
        else:
            self._stuck_hits = 0

        if cmd.lateral_error > float(self.get_parameter('replan_tolerance').value) \
                and cmd.remaining > 0.6:
            self._lat_bad += 1
            # 判据从 8 帧（0.4 s）放宽到 40 帧（2 s）：
            # 换挡瞬间参考点会前跳几米，横向偏差会**瞬时**冲到 1.5 m 以上；8 帧
            # 的窗口根本滤不掉它，于是一次入位里被触发 4~5 次「横向偏差持续」重
            # 规划，每次都要从头再来一遍，既拖时间又把终点误差重新放大。
            # 真正的跑偏会持续好几秒，2 s 的窗口足够区分。
            if self._lat_bad >= 40:
                self.get_logger().warn(f'横向偏差持续 {cmd.lateral_error:.2f} m，重新规划')
                self._lat_bad = 0
                self._plan(now)
                return
        else:
            self._lat_bad = 0

        if cmd.done:
            if self._escape_pending:
                # 脱困倒退走完 → 重新规划入位
                self._escape_pending = False
                self._plan(now)
                return
            if self._reenter:
                # 上一次是「退回预备位姿」，到了就重新做一遍入位
                self._reenter = False
                self._plan(now)
                return
            self._finish(now)
            return

        if cmd.remaining < 0.15:
            gx, gy, gth = self.target.goal_pose
            err = math.hypot(gx - self.pose[0], gy - self.pose[1])
            yaw_err = abs(wrap(gth - self.pose[2]))
            if err > DONE_XY_TOL or yaw_err > DONE_YAW_TOL:
                self._dock_retry += 1
                if self._dock_retry <= DOCK_RETRY_MAX:
                    # 航向差太大时，在车位里原地微调是修不回来的：转向已经打满，
                    # 没有多余自由度同时纠横向和航向。物理世界的 APA 这时也是
                    # 「往前开出来、重新倒一次」，所以这里先规划回预备位姿。
                    if yaw_err > math.radians(12.0) and self._recover_left > 0:
                        out = self.planner.plan(
                            self._rear(self.pose),
                            self._rear(self.target.staging_pose))
                        if out.success:
                            self._recover_left -= 1
                            self.path = out.path
                            self.follower.set_path(self.path)
                            self._follow_start = now
                            self._reenter = True
                            self.publish_path()
                            self.get_logger().info(
                                f'入位偏差 {err:.2f} m / '
                                f'{math.degrees(yaw_err):.1f}°，先退回预备位姿'
                                f'（{out.length:.1f} m）重新入位')
                            return
                    self.get_logger().info(
                        f'入位微调（第 {self._dock_retry} 次）：'
                        f'横向 {err:.2f} m / {math.degrees(yaw_err):.1f}°')
                    self._plan(now)
                    return
                self.get_logger().warn(
                    f'入位微调次数用尽（{err:.2f} m / '
                    f'{math.degrees(yaw_err):.1f}°），按当前位姿结束')
                self._finish(now)
                return

        tw = Twist()
        tw.linear.x = float(cmd.v)
        tw.angular.z = float(cmd.omega)
        # [同学 C 修复] 入位段原来**没有任何护栏**：激光只在巡航和 ALIGN 里过一遍，
        # 真正的倒车入库是裸奔的，控制器一跑偏就直接剐邻车。这里补两道，都很轻：
        #   1. 行人：只在人贴到车身旁边才停（半径小，避免路过车位口就打断揉库）；
        #   2. 侧向间隙：只在真的要剐上（默认 0.10 m）才停，不要求纵向留空，
        #      所以车位后墙的 0.33 m 不会误触发。
        if self._ped_in_front(float(self.get_parameter('ped_yield_park').value)):
            self.stop()
            if now - self._ped_log > 1.5:
                self._ped_log = now
                self.get_logger().info('入位让行：行人贴近车体，暂停')
            return
        if not self._side_guard_ok(now, gear=cmd.gear, remaining=cmd.remaining):
            return
        self.pub_cmd.publish(tw)
        if now - self._last_cmd_log > 2.0:
            self._last_cmd_log = now
            d = getattr(self.follower, 'dbg', {}) or {}
            # 诊断量：参考点 yaw 与自车 yaw 的差（phi）、横向垂距（e）、指令曲率。
            # 换挡瞬间 phi 若突然变成几十度，说明参考点错位；κ 长期贴限幅
            # 说明反馈饱和。这两者是最常见的两类「倒车越走越偏」。
            # `欠转比 = ω实测 / ω指令` 是执行器标定的直接观测量：稳态值应等于
            # 1/steer_gain（现在 1/1.04 = 0.96）。注意它只是**单帧差分**，噪声极大，
            # 只能看长期趋势；要精确标定请用台架 `scripts/bench_steer.py`。
            _meas = self._yaw_rate_meas
            _ratio = (_meas / cmd.omega) if (abs(cmd.omega) > 1e-3 and _meas) \
                else float('nan')
            self.get_logger().info(
                f'跟踪中 v={cmd.v:.2f} m/s ω={cmd.omega:+.2f} rad/s '
                f'ω实测={(0.0 if _meas is None else _meas):+.2f} '
                f'(欠转比={_ratio:.2f}) '
                f'剩余={cmd.remaining:.2f} m 横向偏差={cmd.lateral_error:.2f} m '
                f'档位={"D" if cmd.gear > 0 else "R"} 风险={self.risk} | '
                f'参考yaw={math.degrees(d.get("thr", 0.0)):+.1f}° '
                f'φ={math.degrees(d.get("phi", 0.0)):+.1f}° '
                f'e={d.get("e", 0.0):+.2f} κ={d.get("kappa", 0.0):+.3f} '
                f'seg={d.get("seg", 0)} 侧面间隙={self._side_gap:.2f} m')

    def stop(self):
        self.pub_cmd.publish(Twist())

    def _finish(self, now):
        """结束本次泊车。达标才报 DONE，否则报 INCOMPLETE（不谎报成功）。

        [同学 C 本轮] 达标判据新增一条：**整段入位过程中车体到障碍的最小距离
        必须 ≥ `MIN_PARK_GAP`（0.20 m）**。原来只看终点误差 / 航向，于是
        「擦着邻车停进去」也算 DONE —— 物理上剐蹭是安全事件，不该被终点精度掩盖。
        """
        goal = self.target.goal_pose
        err_xy = math.hypot(self.pose[0] - goal[0], self.pose[1] - goal[1])
        err_yaw = abs(wrap(self.pose[2] - goal[2]))
        gap_ok = self.min_park_gap >= MIN_PARK_GAP
        ok = err_xy <= DONE_XY_TOL and err_yaw <= DONE_YAW_TOL and gap_ok
        self.state = 'DONE' if ok else 'INCOMPLETE'
        self.finish_time = now
        self.stop()
        if not ok:
            why = []
            if err_xy > DONE_XY_TOL or err_yaw > DONE_YAW_TOL:
                why.append(f'{err_xy:.2f} m / {math.degrees(err_yaw):.1f}°'
                           f'（阈值 {DONE_XY_TOL:.2f} m / '
                           f'{math.degrees(DONE_YAW_TOL):.0f}°）')
            if not gap_ok:
                why.append(f'最小车体间隙 {self.min_park_gap:.2f} m'
                           f'（阈值 {MIN_PARK_GAP:.2f} m）')
            self.get_logger().warn('泊车未达标：' + '；'.join(why) + '，状态标记为 INCOMPLETE')
        self._log_done()

    def publish_mode(self, mode):
        m = String()
        m.data = mode
        self.pub_mode.publish(m)

    def _log_done(self):
        goal = self.target.goal_pose
        err_xy = math.hypot(self.pose[0] - goal[0], self.pose[1] - goal[1])
        err_yaw = abs(wrap(self.pose[2] - goal[2]))
        gap = (self.min_park_gap if self.min_park_gap < 90 else float('nan'))
        self.get_logger().info(
            f'泊车完成：车位 {self.target.id}（{self.target.kind}），'
            f'终点误差 {err_xy:.2f} m / {math.degrees(err_yaw):.1f}°，'
            f'车体最小间隙 {gap:.2f} m（阈值 {MIN_PARK_GAP:.2f} m），'
            f'总耗时 {self.finish_time - self.start_time:.1f} s')
        self.plan_info.update({'err_xy': round(err_xy, 3),
                               'err_yaw_deg': round(math.degrees(err_yaw), 2),
                               'side_gap': (round(self._side_gap, 3)
                                            if self._side_gap < 90 else None),
                               'min_gap': (round(gap, 3)
                                           if gap == gap else None),
                               'total_time': round(self.finish_time - self.start_time, 1)})
        try:
            new = not os.path.exists(self.eval_csv)
            with open(self.eval_csv, 'a', encoding='utf-8') as f:
                if new:
                    f.write('time,stall,kind,region,err_xy,err_yaw_deg,path_len,'
                            'reverse,switch,plan_time,total_time,risk_at_end,min_gap\n')
                f.write(f'{time.strftime("%Y-%m-%d %H:%M:%S")},{self.target.id},'
                        f'{self.target.kind},{self.target.region},{err_xy:.3f},'
                        f'{math.degrees(err_yaw):.2f},'
                        f'{self.plan_info.get("length", 0):.2f},'
                        f'{self.plan_info.get("reverse", 0):.2f},'
                        f'{self.plan_info.get("switch", 0)},'
                        f'{self.plan_info.get("time", 0)},'
                        f'{self.finish_time - self.start_time:.1f},{self.risk},'
                        f'{gap:.3f}\n')
        except Exception as e:
            self.get_logger().warn(f'写评测 CSV 失败：{e}')

    # ------------------------------------------------------------------ 发布
    def _publish_target_spot(self, stall):
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.point.x = float(stall.cx)
        msg.point.y = float(stall.cy)
        msg.point.z = 0.0
        self.pub_target.publish(msg)

    def publish_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'
        for (x, y, th, *_rest) in self.path:
            ps = PoseStamped()
            ps.header = path.header
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.z = math.sin(th / 2)
            ps.pose.orientation.w = math.cos(th / 2)
            path.poses.append(ps)
        self.pub_path.publish(path)

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'stall': self.target.id if self.target else '',
            'kind': self.target.kind if self.target else '',
            'region': self.target.region if self.target else '',
            'risk': self.risk,
            'pose': [round(v, 2) for v in (self.pose or (0, 0, 0))],
            'plan': self.plan_info,
            'blacklist': sorted(self.blacklist),
            'scans': self.scan_count,
        }, ensure_ascii=False)
        self.pub_status.publish(msg)

    def publish_viz(self):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()
        if self._evals is None or time.time() - self._evals_stamp > 1.5:
            self._evals = self.detector.evaluate_all()
            self._evals_stamp = time.time()
        evals = self._evals

        for i, stall in enumerate(self.layout.stalls):
            info = evals.get(stall.id, {'state': 'blocked'})
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'map'
            m.ns = 'stalls'
            m.id = i
            m.type = Marker.LINE_STRIP
            m.action = Marker.ADD
            m.scale.x = 0.06
            m.pose.orientation.w = 1.0
            for (x, y) in stall.rect.corners + [stall.rect.corners[0]]:
                m.points.append(geometry_point(x, y, 0.05))
            target = self.target is not None and stall.id == self.target.id
            state = info.get('state', 'blocked')
            if target:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.4, 1.0, 1.0
            elif state == 'free':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.9, 0.2, 0.9
            elif state == 'occupied':
                m.color.r, m.color.g, m.color.b, m.color.a = 0.95, 0.15, 0.15, 0.95
            else:
                m.color.r, m.color.g, m.color.b, m.color.a = 0.7, 0.7, 0.7, 0.6
            arr.markers.append(m)

        if self.target is not None:
            m = Marker()
            m.header.stamp = now
            m.header.frame_id = 'map'
            m.ns = 'target'
            m.id = 1000
            m.type = Marker.ARROW
            m.action = Marker.ADD
            m.pose.position.x = self.target.cx
            m.pose.position.y = self.target.cy
            m.pose.position.z = 0.3
            m.pose.orientation.z = math.sin(self.target.yaw / 2)
            m.pose.orientation.w = math.cos(self.target.yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = 2.0, 0.35, 0.35
            m.color.r, m.color.g, m.color.b, m.color.a = 0.1, 0.4, 1.0, 0.95
            arr.markers.append(m)
        self.pub_marker.publish(arr)

        if not bool(self.get_parameter('publish_free_grid').value) or self.pose is None:
            return
        # 只发布自车周围 30 m 的自由空间证据，避免整张大地图每帧序列化
        det = self.detector
        half = 15.0
        ix0 = max(0, int((self.pose[0] - half - det.xmin) / det.res))
        ix1 = min(det.width, int((self.pose[0] + half - det.xmin) / det.res))
        iy0 = max(0, int((self.pose[1] - half - det.ymin) / det.res))
        iy1 = min(det.height, int((self.pose[1] + half - det.ymin) / det.res))
        if ix1 <= ix0 or iy1 <= iy0:
            return
        hits = det.hits[iy0:iy1, ix0:ix1]
        free = det.free[iy0:iy1, ix0:ix1]
        data = np.full(hits.shape, -1, dtype=np.int8)
        data[free] = 0
        data[hits >= det.occ_thresh] = 100
        grid = OccupancyGrid()
        grid.header.stamp = now
        grid.header.frame_id = 'map'
        grid.info.resolution = det.res
        grid.info.width = int(ix1 - ix0)
        grid.info.height = int(iy1 - iy0)
        grid.info.origin.position.x = det.xmin + ix0 * det.res
        grid.info.origin.position.y = det.ymin + iy0 * det.res
        grid.info.origin.orientation.w = 1.0
        grid.data = data.ravel().tolist()
        self.pub_free.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = ParkingPlanner()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
