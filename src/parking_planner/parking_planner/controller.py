"""路径跟踪：纯跟踪（pure pursuit）+ 纵向速度规划 + 换挡处理。

控制量约定（实测标定，见 docs/04_接口与联调记录.md）：
* `/cmd_vel.linear.x` 是**带符号车速**（负值倒车，实测可用）；
* `/cmd_vel.angular.z` 是**期望偏航角速度**，Gazebo 的 AckermannSteering 内部
  换算成前轮转角（实测 w=0.3 rad/s 时实际偏航率约 0.24 rad/s，约 20% 转向不足，
  闭环跟踪可以自然修正，另有 `steer_gain` 前馈补偿）。
"""

import math

from .geometry import wrap


class FollowCommand:
    __slots__ = ("v", "omega", "done", "idx", "remaining", "lateral_error", "yaw_error",
                 "target_speed", "gear", "stuck")

    def __init__(self, v=0.0, omega=0.0, done=False, idx=0, remaining=0.0,
                 lateral_error=0.0, yaw_error=0.0, target_speed=0.0, gear=1,
                 stuck=False):
        self.v = v
        self.omega = omega
        self.done = done
        self.idx = idx
        self.remaining = remaining
        self.lateral_error = lateral_error
        self.yaw_error = yaw_error
        self.target_speed = target_speed
        self.gear = gear
        self.stuck = stuck


class PathFollower:
    def __init__(self, wheelbase=2.70, max_steer=0.49,
                 v_max=0.45, v_min=0.25, a_lat=0.8, a_dec=0.6,
                 lookahead_min=0.8, lookahead_gain=0.4,
                 goal_xy_tol=0.35, goal_yaw_tol=math.radians(10.0),
                 steer_gain=1.40, shift_hold=0.8,
                 k_lat=0.6, k_head=1.2, k_rate=0.8, steer_rate=0.30,
                 preview=0.0, smooth=0.0, k_int=0.0, int_limit=0.40,
                 tail_anchor=True, ld_floor=0.0, flip_preload=0.30,
                 ref_ahead=None):
        self.wheelbase = float(wheelbase)
        self.max_steer = float(max_steer)
        self.v_max = float(v_max)
        self.v_min = float(v_min)
        self.a_lat = float(a_lat)
        self.a_dec = float(a_dec)
        self.lookahead_min = float(lookahead_min)
        self.lookahead_gain = float(lookahead_gain)
        self.goal_xy_tol = float(goal_xy_tol)
        self.goal_yaw_tol = float(goal_yaw_tol)
        self.steer_gain = float(steer_gain)
        self.shift_hold = float(shift_hold)
        self.k_lat = float(k_lat)
        self.k_head = float(k_head)
        # 偏航率阻尼（微分项）：实测主干道上的 ω 是一条**振幅持续增长**的振荡
        # （0.02→0.04→0.09→0.13→贴限幅，周期约 4.5 s），属于闭环不稳定而不是
        # 扰动 —— 只调 k_lat/k_head 改不了符号。原因是转向执行（插件关节有
        # 3 rad/s 的速度上限）和 20 Hz 采样带来相位滞后。加一项偏航率误差反馈
        # 相当于给摆振加阻尼，这是消除这类极限环的标准做法。
        self.k_rate = float(k_rate)
        # 横向偏差的**积分**项（只在倒车段用）。
        # 真机日志证明倒车弧上存在一个**常值曲率偏差**：车辆实际偏航率只有
        # 指令的 0.65~0.80（Gazebo 的 AckermannSteering 在低速下转向不足），
        # 这不是噪声而是确定性扰动。纯 P/PD 对常值扰动只能把它压成**线性增长**
        # 的横向偏差 —— 实测 W7 在 9 m 的倒车弧上从 0.06 m 稳涨到 1.05 m，
        # 而航向差 φ 反而逐渐收敛到 0，正是「两条同心圆弧、半径不同」的形态。
        # 积分项把这个常值偏差积分成额外的曲率指令，稳态偏差才能被消掉。
        # 上限 `int_limit` 防积分饱和（0.40 m·s 相当于最多 0.06 rad/m 的补偿）。
        self.k_int = float(k_int)
        self.int_limit = float(int_limit)
        self._e_int = 0.0
        # 转向速率上限 (rad/s²)：防止定位跳变时方向盘从死左瞬间甩到死右
        self.steer_rate = float(steer_rate)
        # 前视预瞄距离 (m)：>0 时改用**预瞄纯跟踪** κ = 2·sin(α)/L_p（α 为车头指向
        # 到「垂足前方 preview 米处路径点」的夹角），0 = 旧的 e/φ 反馈律。
        # 零预瞄下 e 与 φ 同相位推舵，叠加执行滞后必然振荡（主干道左右摇摆）。
        self.preview = float(preview)
        # 指令一阶低通时间常数 (s)：0 = 关闭。用于压掉定位噪声引起的舵抖动。
        self.smooth = float(smooth)
        self._omega_lp = None
        self._last_omega = None
        # 末段收尾行为（路线段 True / 入位段 False）：
        #   True  —— remaining<6 时预瞄线性收到 1.0 m，且预瞄够不到时锚定在
        #            **路径终点**上（「路线 → 预备位姿」的交接对准靠它）。
        #   False —— 不收缩也不锚定，预瞄够不到（要跨换挡点 / 超出本段剩余）
        #            就退回 e/φ 反馈律。入位段的定位横向残差（实测峰值 0.29 m）
        #            会被塌缩的 L_d 按 2/L_d 放大成顶满的 κ（真机 E2 末段
        #            la_d 1.9→1.0、e 从 0.08 涨到 0.41、κ 在 ±0.202 乱撞、
        #            终点 0.44 m 且剐蹭）—— 宁可用温和的反馈律收尾。
        self.tail_anchor = bool(tail_anchor)
        # 预瞄纯跟踪的分母下限（m）：锚定在终点上时 Ld 会随车逼近而塌缩，
        # 增益 2/L_d 随之发散（0.3 m 定位残差在 Ld=0.5 时被放大成 κ>1）。
        # 入位段取 1.2（增益封顶 1.67），路线段取 1.0（与预瞄收缩的下限一致），
        # 0 = 不设下限（旧行为）。
        self.ld_floor = float(ld_floor)
        # [同学 C 本轮] **变号点前瞻预偏转**（m）：垂足距「同档位曲率变号点」不足
        # 这个距离时，曲率前馈提前从当前段向变号后的段线性过渡。
        # E2 收尾弧（C|C，κ +0.165→-0.165 突变）0.26 m 间隙损失的根因：变号瞬间
        # 路径曲率要求突翻，而车带着旧弧的航向变化率惯性 + 执行器翻转
        # （δ 速率 3 rad/s + 一阶滞后 0.25 s）需要 ~0.6 s —— 无噪声 trace 实测
        # φ 在 1 s 内从 -0.2° 爆发到 -5.8°，随后 κ 顶死限幅（0.202 vs 需求
        # 0.165，反馈超额仅 0.037）只能以 ~0.9°/s 慢慢追，8 s 才收回，期间
        # e 积累到 0.21 m，把收尾弧中段的间隙从规划 +0.319 吃到 +0.06。
        # 预偏让曲率指令在变号点前 flip_preload 米内提前翻转：跨点时航向
        # 变化率已就位，φ 不爆发；预偏本身只积累 Δκ·D/2（0.33×0.30/2≈2.9°）
        # 的小超前，交给下一段的反馈收敛。0 = 关闭。
        self.flip_preload = float(flip_preload)
        # [同学 C 本轮 §6.28] **运动学侧偏（β）补偿**：弧线运动时**非后轴**参考点
        # 的速度不在航向上，而是偏 β = atan(κ·ref_ahead)（κ=0.165 的弧上 ≈12.6°，
        # 真机 /odom_gt 实测 -12°~-15°，与 atan(0.165×1.35) 精确吻合 —— 是几何
        # 必然，不是轮胎侧滑）。反馈律若拿「航向误差 φ」当「速度方向误差」，闭环
        # 会把 φ 压到 0 而速度仍偏切线 β —— e 以 sin(β)≈0.17/m 单调发散（真机
        # E2 D 段实测；W7「倒车弧同心圆、e 线性涨到 1.05 m」同根因）。修复：航向
        # 项改用**速度方向误差** φ_v = φ + gear·β̂（β̂ 由上一拍指令曲率估计），
        # 纯跟踪的 α 同步改为从速度方向起算（α − gear·β̂）。
        # [同学 C 本轮 §6.29] `ref_ahead` 的语义升级为「**路径参考点到后轴的
        # 距离**」，一个参数同时驱动两件事：
        #   · **反馈位姿换算**（update 入口）：输入 pose 恒为底盘中心（定位
        #     输出），参考点 = 中心后方 (L/2 − ref_ahead) 米。中心参考
        #     （ref_ahead = L/2，路线段的默认）时换算量为 0 —— 完全兼容
        #     旧行为；后轴参考（ref_ahead = 0，入位段自 §6.29 起）时换算到
        #     后轴，与 hybrid_astar 输出的后轴路径同一坐标系；
        #   · **β̂ = atan(κ_exec·ref_ahead)**：参考点是后轴时自动为 0 ——
        #     后轴速度恒沿车体纵轴，β 补偿无需分支、自然关闭。这正是把
        #     E2 的「C|C 变号点速度方向瞬跳 2β≈25°」从根上消掉的那一刀。
        # None = 轴距一半（中心参考，兼容旧调用）。
        self.ref_ahead = (float(ref_ahead) if ref_ahead is not None
                          else self.wheelbase * 0.5)
        self._kappa_exec = 0.0

        self.path = []
        self.idx = 0
        self.gear = 1
        self._hold_left = 0.0
        # [B3 修复 · 换挡起步自检] 换挡刚结束、起步的第一拍标记（见 update）。
        self._shift_fresh = False
        self._prev_idx = 0
        self._lat = 0.0
        self._yaw_err = 0.0
        # 进度看门狗：路径进度 remaining 连续 STALL_LIMIT 秒不减少就报 stuck。
        # 没有它的话，「换挡点门限没被满足 → 永远不换挡 → 一直朝一个方向开」
        # 这类死锁会表现为车在原地/原地打圈几十秒，看起来就是「停驻」。
        self._last_remaining = None
        self._stall_time = 0.0
        self._last_omega = None
        self.STALL_LIMIT = 5.0
        # 换挡门限：当前位置的垂足沿路径到换挡点还剩多少**弧长**才允许换挡。
        # 取值是个折中：太大 → 换挡时参考点离车太远，φ 被人为抬高（弧长 1 m、
        # 最大曲率下约 10°）；太小 → 垂足停在换挡点前一段上就永远够不到，
        # 退化成「一直朝一个方向开」的死锁（实测 0.35 m 时 P1/B3 直接卡死）。
        # 0.70 m 在两组回归里都成立。
        self.shift_gate_arc = 0.70
        # [同学 C 修复] **首换挡窄门限**。0.70 m 的门限有一个盲区：路径的第一段
        # 本来就不长（起步小段）时，「到换挡点的弧长 ≤ 0.70」在车还没开出多少的
        # 时候就成立 —— 换挡过早触发，前进入剩下大半没开，参考点直接落到换挡
        # 之后的弧里。真机 W9：t=0 换挡、第一拍 φ=−14.3°、κ 顶死限幅、横向峰值
        # 1.02 m（§6.23⑤2）；离线 W9（交接横偏 0.15 m）的起步头是一条 0.92 m
        # 的弧，常规门限在只开出 0.22 m 时就换挡，随后 e 一路涨到 0.53 m 卡死。
        # 所以**首段短于微头阈值**（与规划侧 `MIN_SEG_LEN`=1.2 同一判据，
        # `strip_micro_head` 保留下来的就是这种头）时，第一次换挡改用窄门限：
        # 车先实际开出首段的一半、且距换挡点不超过 0.15 m，才允许换。
        # 首段长于阈值的车位（两段式、平行揉库）不受影响，仍走 0.70 常规门限。
        self.micro_head_len = 1.2
        self.first_shift_arc = 0.15
        # 首个换挡点下标与首段弧长，set_path 时重算
        self._first_cusp_idx = None
        self._first_leg = 0.0
        # 全路径换挡次数（set_path 时重算）。单换挡路径的换挡点之后**没有任何
        # 同档位段**，档位过滤后的垂足不可能跳过它 —— 首换挡可以用窄门限而
        # 不必担心「垂足跳过去、换挡永远不触发」的死锁（那种死锁需要换挡点
        # 之后存在空间上重叠的同档位支路，只出现在多换挡的揉库路径里）。
        self._n_cusp = 0
        # 垂足搜索的前视上限（弧长，m）。换挡点附近路径会掉头，空间上重叠的
        # 后段极易被误判成「最近的参考点」；配合「只认当前档位线段」的过滤后，
        # 这个上限只需兜住「大幅横向偏差时垂足确实很远」的情况，故取 3 m。
        self.proj_max_ahead = 3.0
        # 最近一次投影落点（段起点下标 / 段内参数），换挡门限要用它算弧长
        self._proj_seg = 0
        self._proj_t = 0.0
        # 最近一次 _lookahead 的「窗口跨曲率变号」标志（见 _lookahead 内的说明）
        self._la_flip = False
        # 最近一帧的跟踪中间量（只用于诊断日志，不参与控制）
        self.dbg = {}

    # ------------------------------------------------------------------ 路径管理
    def set_path(self, path, start_idx=0):
        """path: [(x, y, yaw, direction, curvature), ...]

        也接受 3 元组 (x, y, yaw)（全局路线的点），自动补成「前进 + 零曲率」。
        """
        norm = []
        for p in path:
            if len(p) >= 5:
                norm.append(tuple(p))
            else:
                norm.append((p[0], p[1], p[2], 1, 0.0))
        self.path = norm
        self.idx = max(0, min(start_idx, len(self.path) - 1))
        self.gear = self.path[self.idx][3] if self.path else 1
        self._hold_left = 0.0
        self._shift_fresh = False
        self._prev_idx = self.idx
        self._last_remaining = None
        self._stall_time = 0.0
        self._last_omega = None
        # 低通/速率限幅的状态必须跟着路径复位：否则新路径的第一帧会带着旧路径
        # 的 ω（方向可能完全相反），被速率限幅拖成一条斜爬的错舵。
        self._omega_lp = None
        # β 补偿的指令曲率估计同样复位（新路径起步 κ=0）
        self._kappa_exec = 0.0
        # 积分项也必须复位：新路径的起点偏差不是「持续扰动」，累积下来会顶到限幅
        self._e_int = 0.0
        # 投影复位：remaining() 依赖它，必须是「从头开始」的状态
        self._proj_seg = 0
        self._proj_t = 0.0
        # 首个换挡点与首段弧长（供首换挡窄门限用，见 __init__ 的说明）
        self._first_cusp_idx = None
        self._first_leg = 0.0
        self._n_cusp = sum(
            1 for a, b in zip(norm, norm[1:]) if a[3] != b[3])
        for k in range(1, len(norm)):
            if norm[k][3] != norm[0][3]:
                self._first_cusp_idx = k
                break
        if self._first_cusp_idx is not None:
            self._first_leg = sum(self._seg_len(k)
                                  for k in range(self._first_cusp_idx))

    def remaining(self):
        """从当前位置的**垂足**沿路径到终点的弧长（m）。

        旧实现只从 `idx+1` 往后累加，把「垂足到 idx+1 的那一小段」丢掉了；更要紧
        的是它完全依赖索引，索引一旦被前跳（换挡）或超车（横向偏差大时），剩余
        里程就会偏短，于是 `remaining<0.02` 提前成立、车早停半米。现在把当前
        线段的剩余部分算进来，剩余里程只可能偏保守（偏长），不会少算。
        """
        out = (1.0 - self._proj_t) * self._seg_len(self._proj_seg)
        for i in range(self._proj_seg + 1, len(self.path) - 1):
            out += self._seg_len(i)
        return out

    def _seg_len(self, i):
        if i < 0 or i + 1 >= len(self.path):
            return 0.0
        return math.hypot(self.path[i + 1][0] - self.path[i][0],
                          self.path[i + 1][1] - self.path[i][1])

    def _lookahead(self, seg_i, seg_t, dist, anchor_end=True):
        """从**垂足**沿路径向前 dist 米处的点 (x, y, yaw)，跨档位返回 None。

        为什么需要它：原来参考点只有垂足，等于零预瞄。零预瞄下 e_lat 与 φ 同相位
        地推舵，而转向执行（关节有速度上限）和 20 Hz 采样带来相位滞后，闭环就是
        一个增益过大的一阶系统 —— 实测主干道上 ω 是一条**振幅持续增长**的振荡
        （0.02→0.04→0.09→0.13→贴限幅，周期约 4.5 s），车真的左右摆。
        预瞄等于在航向环里加一个零点：车还没偏就先按前方路径打舵，摆振被压掉。

        anchor_end=True（路线段）：路径剩余不足 dist 时把预瞄点收到**路径终点**
        （剩余 <0.5 m 才退回旧律）。旧写法在最后 2.8 m 直接退回零预瞄，恰好把
        「路线 → 预备位姿」交接前最后几米的横向偏差还给搞大了（实测 0.41 m），
        而交接位姿一差，后面的两段式泊车就要多揉一把、甚至规划失败。

        anchor_end=False（入位段）：同样锚定在路径终点，但有两处收紧——
        剩余 <0.5 m 就退回 e/φ 反馈律（比路线段的 0.5 m 门限更早生效，
        且锚定增益由 update 侧的 ld_floor 封顶），预瞄要**跨换挡点**时
        一律返回 None（锚死在换挡点上会让 Ld 塌缩、2/L_d 增益发散，
        见函数体内的修复说明）。
        """
        n = len(self.path)
        if dist <= 0.0 or n < 2:
            return None
        self._la_flip = False
        i = max(0, min(seg_i, n - 2))
        left = (1.0 - seg_t) * self._seg_len(i)
        # 先看这条路还剩多长（不跨档位），预瞄距离不能超过它
        rest_total = left
        j = i + 1
        gear_cut = False
        flip_full = False
        while j < n - 1:
            if self.path[j + 1][3] != self.gear:
                gear_cut = True
                break
            if self.path[j][4] * self.path[j + 1][4] < -1e-12:
                flip_full = True
            rest_total += self._seg_len(j)
            j += 1
        if rest_total < 0.50:
            return None
        # [同学 C 修复] 预瞄被**换挡点**截断时不能把预瞄点锚死在换挡点上。
        # 真机 E2（15.4 m / 1 换挡，D 段仅 4.1 m）：垂足进入「段剩余 < preview」
        # 的窗口后预瞄点被钉在换挡点，Ld 从 2.4 m 一路塌缩到 0.5 m，纯跟踪
        # 增益 2/Ld 随之发散 —— 0.3 m 的定位横向残差（位姿对照实测峰值 0.29 m）
        # 被放大成反向顶满的 κ（日志 κ=+0.202，而参考曲率是 -0.165），与
        # 预瞄失效帧（落回反馈律的 -0.114）逐帧对撞，D 段末尾 5 s 内把车推偏
        # 0.31 m / 8.7°，换挡后 R 段以大偏差起步、κ 顶满也追不回，横向偏差
        # 单调涨到 1.24 m 触发重规划，最终 91 m / INCOMPLETE。
        if gear_cut and dist > rest_total:
            return None
        if dist >= rest_total:
            if not anchor_end and rest_total < 0.50:
                # 入位段：段末不足 0.5 m 才退反馈（终点锚定的 Ld 塌缩由
                # update 侧的 ld_floor 兜住，见那里的说明）
                return None
            # 锚定/延伸窗口 = 垂足→档位链尽头，正好是上面 rest_total 循环
            # 扫过的范围：变号检测直接用 flip_full。
            if flip_full:
                self._la_flip = True
            # [同学 C 修复] 锚定必须**直接返回路径终点**，不能压 dist=rest_total
            # 后继续走下面的逐段步行。步行要求最后一段满足 `L >= rest` 的精确
            # 比较，而 rest 是「left + 逐段累加」再减回去的浮点和，与最后一段
            # 相差 ±1e-16：差为正时步行走到 j=n-1 直接耗尽 → return None ——
            # 预瞄在「剩余 < preview」的整段窗口里死亡（la_d=0 连续一百多帧），
            # 末段退回从未调过参的 e/φ 反馈律，κ 长期顶死限幅、航向残差 12°
            # （离线 seed5：stuck 13.4°；seed3/seed8 同形态）。
            # gear_cut=False 时档位链必然延伸到路径末点，锚点就是 path[-1]。
            p_end = self.path[n - 1]
            # [同学 C 本轮] 入位段（anchor_end=False）不锚死在终点上，而是沿
            # **末端行进方向**把预瞄点虚拟延伸 (dist - rest_total)。锚死时 Ld
            # 随车逼近终点塌缩，方位角噪声 e/Ld_true 被放大 —— ld_floor 只
            # 封顶增益 2/L_d，封不住角度项（seed5 末段：la_d 0.5~0.7、
            # 0.2 m 定位残差折成 ±18° 的 α 抖动、κ 逐帧在 ±限幅乱撞、
            # 末端航向残差 22°）。延伸后 Ld 稳定在预瞄距离附近，且虚拟车头
            # 自然对准「终点航向」——倒车入位末段的收尾正好需要这个对准。
            # 末端行进方向 = 终点 yaw；末段是倒车时车速反向，再转 π。
            # [同学 C 本轮] 延伸段还要**沿末端曲率弯**（一阶弧偏移
            # κ·ext²/2·法线）：直线延伸时预瞄点落在行进方向的延长线上，
            # 而倒车末段车头恰好背对行进方向 —— α 恒≈0、k_pp 恒≈0，
            # 收尾弧的转弯需求在预瞄里凭空消失（E2 收尾弧 3 m / κ=0.165，
            # 末段 k_pp 实测 0~0.05，φ 残留 9~10° 触发 stuck）。弯起来后
            # 预瞄点落在「弧的延长线」上，切弦角恢复 κ·Ld/2，纯跟踪
            # 自然给出 k_pp ≈ κ_ref。
            if not anchor_end and not gear_cut:
                yaw_e = p_end[2]
                if p_end[3] < 0:
                    yaw_e = wrap(yaw_e + math.pi)
                ext = dist - rest_total
                k_end = p_end[4] if len(p_end) >= 5 else 0.0
                if p_end[3] < 0:
                    k_end = -k_end
                off = k_end * ext * ext * 0.5
                return (p_end[0] + math.cos(yaw_e) * ext
                        - math.sin(yaw_e) * off,
                        p_end[1] + math.sin(yaw_e) * ext
                        + math.cos(yaw_e) * off, p_end[2])
            return p_end[0], p_end[1], p_end[2]
        # [同学 C 本轮] **预瞄窗口跨曲率变号**检测：垂足与预瞄点之间的路径
        # 若从一段弧跨进反号弧（E2 的 C|C 收尾：倒车弧 κ=-0.165 → 收尾弧
        # κ=+0.165），两段弧的弦会把曲率平均掉 —— 实测 k_pp 只有 +0.03 而
        # 段需求 -0.165，φ 以约 4°/s 单调攀升 12°，3 m 收尾弧顶死限幅也
        # 追不回。置 `_la_flip` 让 update 弃用 k_pp、退回前馈+航向反馈。
        # 只查「垂足→预瞄点」窗口内的点对：rest_total 循环扫到档位链尽头，
        # 变号点若在预瞄点之后，那是下一窗口的事，提前退律会把整段倒车弧
        # 主体赶出纯跟踪（噪声抑制正是靠它）。
        seg_flip = self.path[i][4] * self.path[i + 1][4] < -1e-12
        if left >= dist:
            if seg_flip:
                self._la_flip = True
            L = self._seg_len(i)
            u = seg_t + (dist / L if L > 1e-9 else 0.0)
            dth = wrap(self.path[i + 1][2] - self.path[i][2])
            px = self.path[i][0] + (self.path[i + 1][0] - self.path[i][0]) * u
            py = self.path[i][1] + (self.path[i + 1][1] - self.path[i][1]) * u
            return px, py, wrap(self.path[i][2] + dth * u)
        if seg_flip:
            self._la_flip = True
        rest = dist - left
        j = i + 1
        while j < n - 1:
            if self.path[j + 1][3] != self.gear:
                break
            if self.path[j][4] * self.path[j + 1][4] < -1e-12:
                self._la_flip = True
            L = self._seg_len(j)
            if L >= rest:
                u = rest / L if L > 1e-9 else 0.0
                dth = wrap(self.path[j + 1][2] - self.path[j][2])
                px = self.path[j][0] + (self.path[j + 1][0] - self.path[j][0]) * u
                py = self.path[j][1] + (self.path[j + 1][1] - self.path[j][1]) * u
                return px, py, wrap(self.path[j][2] + dth * u)
            rest -= L
            j += 1
        return None

    def _flip_preload_ff(self, seg_i, seg_t, gear, k_ref):
        """变号点前瞻预偏转：返回混合前馈（指令语义），不在窗口内返回 None。

        扫描只沿**当前档位链**（换挡点截止）：换挡处的曲率变号有 shift_hold
        停车兜着，预偏无意义。零曲率点跳过 —— 弧-直线-反号弧的混合起点
        天然是 0（垂足在直线上时 k_ref 本来就 ≈0）。
        变号边界取 path[flip_j]（新符号的第一个点）：垂足贴到边界时权重
        恰好到 1，与垂足跨点后 k_ref 的突变为新符号全值**无缝衔接**。
        """
        n = len(self.path)
        if n < 3 or self.flip_preload <= 0.0:
            return None
        last_k = self.path[seg_i][4] if len(self.path[seg_i]) >= 5 else 0.0
        flip_j = None
        j = seg_i + 1
        while j < n - 1:
            if self.path[j + 1][3] != self.gear:
                break
            kj = self.path[j][4] if len(self.path[j]) >= 5 else 0.0
            if abs(kj) > 1e-9:
                if abs(last_k) > 1e-9 and kj * last_k < 0:
                    flip_j = j
                    break
                last_k = kj
            j += 1
        if flip_j is None:
            return None
        d_flip = (1.0 - seg_t) * self._seg_len(seg_i)
        for m in range(seg_i + 1, flip_j):
            d_flip += self._seg_len(m)
        if d_flip > self.flip_preload:
            return None
        k2_path = self.path[flip_j][4]
        k2 = -k2_path if gear < 0 else k2_path
        w = max(0.0, min(1.0, 1.0 - d_flip / self.flip_preload))
        return k_ref + w * (k2 - k_ref)

    def _project(self, x, y, window=12):
        """把当前位置投影到路径折线上，返回 (段起点下标, 段内参数 t, 横向偏差, 路径 yaw)。

        [同学 C 修复] 旧实现用「离哪个采样点最近」当参考点，`e_lat` 其实是到那个
        **点**的距离，而不是到路径的垂距。一旦参考点被前跳，`e_lat` 就被「沿路径
        方向的位移」污染：真机日志里换挡后 1.3 m 行程内它从 0.26 m 涨到 1.51 m，
        几何上不可能——而控制器把这个假误差当真反馈，直接把曲率顶到限幅。
        改成投到**线段**上取有符号垂距后，e_lat 与 phi 都是干净的几何量。

        搜索窗按**弧长**限制（不是按点数）：换挡点处路径会掉头，空间上重叠的
        「后段」往往比「前段」离车更近，纯取最近段会让参考点一次跳 2 m 以上。
        实测（加了 dbg 日志之后）：`φ` 在换挡瞬间从 −9° 跳到 −30°、`seg` 从 13
        跳到 25，曲率立刻贴限幅，倒车就此发散。把前视限制在 1 m 弧长内即可。

        起点允许回看一段：换挡点附近车可能略在参考点之前，只准向前取点会人为
        制造一个十几度的假航向误差（倒车反馈的收敛域只有约 ±20°，经不起这个）。
        """
        n = len(self.path)
        if n < 2:
            thr = self.path[0][2] if n else 0.0
            return 0, 0.0, 0.0, thr
        i0 = max(0, min(self.idx - 1, n - 2))
        # 前视按弧长截断（同时不超过 window 段）
        limit = min(n - 2, self.idx + window)
        i1, acc = self.idx, 0.0
        # `idx` 若被外部/历史状态推到 n-1（换挡点正好在末段），这里必须夹回来，
        # 否则 range(i0, i1+1) 会访问 path[n]。
        i1 = min(i1, n - 2)
        while i1 < limit and acc < self.proj_max_ahead:
            acc += self._seg_len(i1)
            i1 += 1
        best = None
        for i in range(i0, i1 + 1):
            # 只认「当前档位」的线段：换挡点前后两段在空间上重叠，若不加这个
            # 过滤，换挡瞬间参考点会跳到掉头后的那一段上去（实测 seg 13→25、
            # φ 从 −9° 跳到 −30°，曲率立刻贴限幅）。跨换挡点的那一段
            # （一端旧档、一端新档）两边都放行，保证过渡连续。
            if self.path[i][3] != self.gear and self.path[i + 1][3] != self.gear:
                continue
            ax, ay = self.path[i][0], self.path[i][1]
            dx = self.path[i + 1][0] - ax
            dy = self.path[i + 1][1] - ay
            seg2 = dx * dx + dy * dy
            if seg2 < 1e-12:
                continue
            t = ((x - ax) * dx + (y - ay) * dy) / seg2
            t = max(0.0, min(1.0, t))
            px, py = ax + dx * t, ay + dy * t
            d = math.hypot(x - px, y - py)
            # 需要「明显更近」才换段：平行车位的揉库路径会回头，两条支路相距
            # 只有 2 m 左右，若纯取最近段，会在回头弯上跳到另一条支路、把剩余
            # 里程算短（实测 B1 车位因此提前收车、终点误差 8.6 m）。
            # [同学 C 实测] 门槛抬到 0.10 m 并没有改善 W/E 两排的收敛
            # （W7 执行间隙反而从 +0.03 掉到 +0.01），所以保持 0.02。
            if best is None or d < best[0] - 0.02:
                dth = wrap(self.path[i + 1][2] - self.path[i][2])
                best = (d, i, t, px, py, wrap(self.path[i][2] + dth * t))
        if best is None:                      # 路径点全部重合，退化处理
            p = self.path[min(self.idx, n - 1)]
            thr = p[2]
            return self.idx, 0.0, -math.sin(thr) * (x - p[0]) + math.cos(thr) * (y - p[1]), thr
        _d, i, t, px, py, thr = best
        e_lat = -math.sin(thr) * (x - px) + math.cos(thr) * (y - py)
        return i, t, e_lat, thr

    # -------------------------------------------------------------------- 主入口
    def update(self, pose, dt, speed_scale=1.0, yaw_rate=None):
        """返回 FollowCommand。pose=(x,y,yaw)；speed_scale 用于风险降速；
        yaw_rate 为实测偏航率（rad/s），用于阻尼项，None 表示不启用。"""
        if not self.path:
            return FollowCommand(done=True)

        x, y, th = pose
        # [同学 C 本轮 §6.29] 换算到**路径参考点系**：pose 恒为中心位姿（定位
        # 输出），参考点 = 中心后方 (L/2 − ref_ahead) 米。中心参考（路线段，
        # ref_ahead=L/2）换算量为 0；后轴参考（入位段，ref_ahead=0）换算到
        # 后轴 —— 之后 _project/_lookahead/dist_end/remaining 全部在参考点
        # 系里进行，与规划器输出的路径同一坐标系。
        _back = self.wheelbase * 0.5 - self.ref_ahead
        if _back > 1e-9:
            x -= math.cos(th) * _back
            y -= math.sin(th) * _back
        # 参考点 = 路径折线上的**垂足**（不是最近的采样点，见 _project 的说明）
        seg_i, seg_t, e_lat, thr_p = self._project(x, y)
        self._proj_seg, self._proj_t = seg_i, seg_t
        if seg_i > self.idx:
            self.idx = seg_i
        self._prev_idx = self.idx
        lat = abs(e_lat)
        remaining = self.remaining()
        # [同学 C 本轮] **末端垂足钳位修正**。车倒车逼近路径终点时，最后一段的
        # 垂足被 t=1 钳在段末点上：车中心明明还差 0.3 m 没到终点，remaining
        # 却已经归零（E2 噪声扫描实测 seed8：rem=0.01 而纵向欠走 0.30）。
        # 于是「走到头 remaining<0.02」分支提前触发，done 判定又用**测量**
        # dist_end（定位漂移虚低 ~0.15 m）→ 提前收车，终点误差 0.31~0.36 m
        # 贴着 0.35 判据擦线。把「车到末点的纵向落后量」补回 remaining，
        # 速度规划与收车判定才基于真实距离（垂足钳在中间段时不存在这个问题：
        # 相邻段共享端点，垂足连续滑过去）。
        if seg_i == len(self.path) - 2 and seg_t >= 1.0 - 1e-6:
            bx, by = self.path[seg_i + 1][0], self.path[seg_i + 1][1]
            ax, ay = self.path[seg_i][0], self.path[seg_i][1]
            d = math.hypot(bx - ax, by - ay)
            if d > 1e-9:
                ux, uy = (bx - ax) / d, (by - ay) / d
                behind = (bx - x) * ux + (by - y) * uy
                if behind > 0.0:
                    remaining += behind

        # 进度看门狗（每帧都算，换挡等待期也算在内）
        if self._last_remaining is not None and remaining > self._last_remaining - 5e-3:
            self._stall_time += dt
        else:
            self._stall_time = 0.0
        self._last_remaining = remaining
        # 收尾阶段（剩余 <0.3 m）允许慢慢磨，不算停滞，避免最后几厘米被误判
        stuck = self._stall_time > self.STALL_LIMIT and remaining > 0.3

        # 换挡：只有当车辆真正走到换挡点附近才执行，避免每帧重复触发
        if self._hold_left > 0.0:
            self._hold_left -= dt
            return FollowCommand(0.0, 0.0, False, self.idx, remaining, lat, 0.0, 0.0,
                                 self.gear, stuck)
        # [B3 修复 · 换挡起步自检] 换挡结束、起步的第一拍（hold 已归零）：垂距
        # 仍 > 0.8 m 说明新弧的起点离车太远（换挡门限被提前触发 / 规划弧悬空），
        # 纯跟踪会以满曲率猛纠、越纠越偏（旧版日志里换挡后 1.3 m 行程内 e_lat
        # 冲到 1.5 m 就是这种形态）。直接报 stuck 交给上层重规划，比硬纠好。
        # 注意 _project 已是段投影 + 档位过滤，lat 是**真**垂距，不会像旧版
        # 最近点判据那样虚高。放在 hold 之后：hold 期间车还没起步，垂距没有
        # 参考意义。
        if self._shift_fresh:
            self._shift_fresh = False
            if lat > 0.8:
                stuck = True
        switch_idx = None
        for i in range(self.idx + 1, min(self.idx + 10, len(self.path))):
            if self.path[i][3] != self.gear:
                switch_idx = i
                break
        if switch_idx is not None:
            sx, sy = self.path[switch_idx][0], self.path[switch_idx][1]
            # 门限按**弧长**判：从当前位置的垂足沿路径到换挡点还有多少米。
            # 原来用欧氏距离 1.2 m，对自相交/回头弯的路径来说弧长可以到 1.8 m，
            # 于是「换挡」发生时车其实还在换挡点前 1.8 m 处——`idx` 一跳，参考点
            # 就飞了。改成弧长后，跳的距离被压到 0.35 m 以内，几何量保持干净。
            # 欧氏 2.0 m 只作为兜底（防止路径自交时在另一条支路上误触发）。
            arc = (1.0 - self._proj_t) * self._seg_len(self._proj_seg)
            for k in range(self._proj_seg + 1, switch_idx):
                arc += self._seg_len(k)
            # 首段比微头阈值还短、或整条路径只有一次换挡时用窄门限（见
            # __init__ 的说明）：起步小段必须先被实际开出（至少一半、距换挡点
            # ≤ first_shift_arc），否则换挡在半路触发，车挂着倒挡去追一条起点
            # 悬在前方的弧 —— φ/e 凭空多出十几度，κ 直接贴限幅。
            # [E2 实测] 首段 1.30 m（> 微头阈值 1.2）的满曲率 C|C 解同样会被
            # 0.70 常规门限截断：只开出 0.65 m 就换挡，航向亏损 11°，而满曲率
            # 弧的反馈余量只有 0.02 rad/m，9.2 m 都纠不回来（末端剩 13.3°）。
            # 单换挡路径不存在「垂足跳过换挡点」的死锁（档位过滤后换挡点后
            # 没有同档位段可跳），所以窄门限对它是安全的。
            # 注意 `arc` 里含着换挡段（switch_idx−1 → switch_idx，约一个采样步）
            # 本身：那段几何上属于换挡后的弧，前进档永远走不到，不扣掉它窄门限
            # 就永远够不着 —— 这正是 P1/B3 当年「垂足停在换挡点前一段上够不到」
            # 死锁的机制。
            gate = self.shift_gate_arc
            if switch_idx == self._first_cusp_idx and (
                    self._n_cusp <= 1
                    or 0.0 < self._first_leg < self.micro_head_len):
                cusp_bit = self._seg_len(switch_idx - 1) if switch_idx >= 1 \
                    else 0.0
                if self._n_cusp <= 1:
                    # [同学 C 本轮] 单换挡路径：门限 = 换挡段全长 + 0.10 m。
                    # 「半段」版（0.5·cusp_bit）要求垂足真的走进换挡段过半，
                    # 但 cusp 两侧的弧近相切时换挡段（switch−1→switch）与前进
                    # 末段几乎共线（E2 实测两条弦相差 <1 cm，而投影换段需要
                    # 0.02 m 的明显更近）——垂足落在哪条弦上由定位噪声决定，
                    # 8 个 seed 只有 2 个踩进 t≥0.5 窗口，其余 arc 恒 ≥ cusp_bit
                    # > 门限，换挡永不触发、车在换挡点前发散。含全段 + 0.10：
                    # 垂足逼近到换挡点前 0.10 m（或落进换挡段任意位置）即触发，
                    # 与弦选择无关。提前量 ≈ 0.10 m + 垂足滞后，比旧 0.36 m
                    # 减半以上；cusp 近相切时两侧支路重叠，这点提前量会被
                    # 倒车段的纯跟踪自然吸收。
                    gate = cusp_bit + 0.10
                else:
                    gate = min(self.first_shift_arc,
                               0.5 * self._first_leg) + cusp_bit
            if arc <= gate and math.hypot(sx - x, sy - y) < 2.0:
                self.gear = self.path[switch_idx][3]
                # [同学 C 修复] `idx` 必须保持 ≤ len-2：`_project` 只搜到 len-2，
                # 而这里原来可能把 idx 直接推到 len-1（换挡点正好在路径末段时）。
                # 之后 `_project` 里 `for i in range(i0, i1+1)` 就会访问 path[n]
                # → IndexError。分段滚动执行（ALIGN 每段只有 2.5 m、点数很少）
                # 必然踩到，一次就能把 planner_node 打死。
                self.idx = min(switch_idx, len(self.path) - 2)
                self._prev_idx = self.idx
                self._hold_left = self.shift_hold
                self._shift_fresh = True   # 起步第一拍做垂距自检（见 update）
                # 换挡：低通和速率限幅状态一并复位。前进/倒车的 ω 符号约定不同，
                # 带着旧状态过换挡点会被低通拖出一个反方向的过渡。
                self._omega_lp = None
                self._last_omega = None
                self._kappa_exec = 0.0   # 停车换挡期间 ω=0，β̂ 归零
                self._e_int = 0.0      # 换挡后积分重新累积
                remaining = self.remaining()
                self._last_remaining = remaining
                self._stall_time = 0.0
                return FollowCommand(0.0, 0.0, False, self.idx, remaining, lat, 0.0, 0.0,
                                     self.gear)

        gear = self.gear

        # 终点判断
        dx_end = self.path[-1][0] - x
        dy_end = self.path[-1][1] - y
        dist_end = math.hypot(dx_end, dy_end)
        yaw_end = wrap(self.path[-1][2] - th)
        self._yaw_err = yaw_end
        # [同学 C 本轮] 提前收车分支从 remaining<0.20 收紧到 0.10。done 用的
        # dist_end / remaining 全是**测量**量：slam 的低频慢漂（实测 E2 峰值
        # 0.29 m）在纵向分量上让垂足虚前移，车明明还差 0.2~0.3 m，测量已经
        # 说「进了容差」—— 8 seed 噪声扫描的终点误差因此贴着 0.30~0.36 m
        # （判据 0.35 擦线）。漂移本身消不掉，能做的是把收车**基线**压低：
        # 垂足必须蹭到离终点 0.10 m 以内才允许提前收，让「基线 + 漂移」的
        # 最坏组合落回判据以内。0.15 以下的减速公式仍给出 ~0.24 m/s 爬行速，
        # 0.10 m 的额外行程 <1 s，不会磨出 stuck。
        if remaining < 0.10 and dist_end < self.goal_xy_tol and abs(yaw_end) < self.goal_yaw_tol:
            return FollowCommand(0.0, 0.0, True, self.idx, remaining, lat, yaw_end, 0.0, gear)
        # 路径走到头（remaining≈0）时的两种收尾：
        #   · 位姿也进了容差 → 正常 done；
        #   · 位姿没进容差 → 以 stuck 上报，让调用方按**当前真实位姿**重规划一段。
        #     为什么不能直接 done：剩下的几厘米常常只是「路径点采样到头了」，车其实
        #     还差半米（真机日志：剩余=0.19 m 却距车位中心 0.85 m），直接 done 就
        #     等于提前收车、车头露在车位外。
        #     也不能继续带着速度往下开（旧行为：离线复现过连续倒车 175 s）。
        # [同学 C 本轮] at_goal 的 xy 容差放宽到 max(goal_xy_tol, 0.30)：
        # 「走到头」是几何事实（垂足已在路径末端），此时收不收车都改变不了
        # 真实位置 —— 漂移虚高的 seed 若在这里报 stuck，重规划也只是从被漂移
        # 污染的测量位姿再走 0.2 m，救不了纵向漂移，反而在车位里多一次启停。
        # yaw 容差维持原值（终点航向是硬指标，放宽会把歪着车头入位合法化）。
        if remaining < 0.02:
            at_goal = (dist_end <= max(self.goal_xy_tol, 0.30)
                       and abs(yaw_end) <= self.goal_yaw_tol)
            return FollowCommand(0.0, 0.0, at_goal, self.idx, remaining, lat,
                                 yaw_end, 0.0, gear, not at_goal)

        # ---------------- 跟踪律 ----------------
        # 纯跟踪在前视距离较大时有稳态滞后（误差 ≈ ld²/2R），在 2.5 m 宽的车位里
        # 会直接把车带偏 0.4 m 以上。这里改为「路径曲率前馈 + 横向/航向反馈」：
        #     κ_cmd = κ_ref - k_e·e - k_φ·φ
        # 其中 e 为横向偏差（左正）、φ 为航向偏差（车头相对路径切线）。
        # 前进用上式；倒车等价于把车头转 180° 后前进，偏航率取反即可。
        # ---------------- 前馈曲率 κ_ref ----------------
        # [同学 C 修复] 优先用路径自带的曲率（Hybrid A* / Reeds-Shepp 路径的第 5 个
        # 字段），并对前方 3 点取均值抑制换挡/离散化跳变。
        # 旧写法只用相邻两点做差分：点距约 0.2 m、yaw 又经过离散化，κ_ref 的噪声
        # 可以到 ±0.1（相当于半径 10 m 的假弯），前馈项就把车一路带偏——
        # 实测入位段横向偏差从 0.01 m 单调涨到 1.2 m，而且 ω 长期贴在限幅上。
        # 全局路线是 3 元组（曲率补 0），此时才退回差分，并把基线放宽到 ±2 点。
        # 只在**前进**时用路径自带曲率：倒车分支的符号约定是「虚拟车头转 180°」，
        # 曲率前馈要取 -k_ref，为不改变已验证过的倒车行为，倒车段仍用差分。
        # [同学 C 修复] 倒车分支现已按 s=−1 统一取反（见下方 fb），符号约定统一，
        # 所以倒车也可以直接用路径自带曲率。差分前馈在「圆弧接直线」处会被
        # ±2 点基线抹平（曲率跳变被摊成斜坡），实测倒车段横向偏差在最后 5 m
        # 从 0.04 m 涨到 1.07 m。
        k_ref = None
        if len(self.path[self.idx]) >= 5:
            # [同学 C 本轮] 只对**同档位点**取均值。旧写法要求 idx..idx+3 四点
            # 全同档才用自带曲率，否则整体退回 ±2 点差分 —— 而差分基线一旦
            # 跨过换挡点就出灾难：cusp 两侧的弧在回扫几何下几乎相切，idx-2
            # 与 idx+2 两点弦长可以缩到厘米级（E2 实测 P19/P23 仅 0.026 m），
            # κ_ref 直接算出 -5.0 的垃圾前馈，指令顶死限幅，车在换挡点前
            # 原地画圈、垂足不再前进、换挡永远不触发（8 seed 中 6 个死锁）。
            # [同学 C 本轮] 还要按**垂足点的曲率符号**过滤：变号点落在均值
            # 窗口里时，-0.165 与 +0.165 的点被平均成 ≈0 的过渡值（E2 实测
            # 垂足过变号点前 k_ref 走 -0.165 → -0.083 → 0 → +0.083 → +0.165，
            # 前馈在最需要它的收尾弧入口凭空消失。
            k0 = self.path[self.idx][4]
            ks = [self.path[j][4] for j in range(
                self.idx, min(self.idx + 4, len(self.path)))
                if self.path[j][3] == gear
                and (abs(k0) < 1e-9 or self.path[j][4] * k0 > -1e-12)]
            if ks and any(abs(k) > 1e-9 for k in ks):
                k_ref = sum(ks) / len(ks)
                # [同学 C 本轮] 倒车段也改用路径自带曲率，但要**整体反号**。
                # 数值核对（`probe_rs_curv.py`，直接打 Reeds-Shepp 的采样）：
                #   前进段 c 与 dθ/ds **同号**（比值 +1.000）
                #   倒车段 c 与 dθ/ds **反号**（比值 −1.000）
                # 而跟踪器要求的 ω = |v|·κ，倒车时 ω = dθ/dt = (dθ/ds)·|v|，
                # 所以 κ_ref = −c。
                # 为什么必须改：倒车段原来走 ±2 点差分，点距 0.20 m、航向 3° 量化，
                # 差分出来的 κ 误差可达 ±12%（0.1747 上下能跳到 0.154 或 0.196），
                # 前馈本身就带误差，再叠加反馈必然越走越偏。真机 E6 的「圆弧没走完
                # 车头就转正了、随后在车位口反向过冲」就是前馈偏大造成的。
                if gear < 0:
                    k_ref = -k_ref
        if k_ref is None:
            a = max(0, self.idx - 2)
            b = min(len(self.path) - 1, self.idx + 2)
            # 差分基线同样不得跨换挡点：跨过去就是上面说的厘米级弦长，
            # κ 被放大到 ±5 以上。两端各自收缩到最近的同档位点。
            # （路线段是 3 元组、没有档位字段，跳过钳位。）
            if len(self.path[self.idx]) >= 5:
                while a < self.idx and self.path[a][3] != gear:
                    a += 1
                while b > self.idx and self.path[b][3] != gear:
                    b -= 1
            if b > a:
                dref = max(1e-3, math.hypot(self.path[b][0] - self.path[a][0],
                                            self.path[b][1] - self.path[a][1]))
                k_ref = wrap(self.path[b][2] - self.path[a][2]) / dref
            else:
                k_ref = 0.0
        # [同学 C 本轮] 变号点前瞻预偏转：垂足距同档位曲率变号点 < flip_preload
        # 时，前馈向变号后的段曲率线性过渡（见 _flip_preload_ff 的推导）。
        # 放在纯跟踪判定之前：预偏窗口 ⊂ _la_flip 窗口（预瞄跨变号点更早
        # 触发），此时走的是降级律/反馈律，k_ref 正是那条律的前馈项。
        if self.flip_preload > 0.0:
            pre = self._flip_preload_ff(seg_i, seg_t, gear, k_ref)
            if pre is not None:
                k_ref = pre
        # e_lat / thr_p 已在函数开头由 _project 给出（垂足处的路径 yaw 与有符号垂距）
        phi = wrap(th - thr_p)
        # [同学 C 本轮 §6.28] β 补偿：把「航向误差」换算成「速度方向误差」。
        # φ_v = φ + gear·β̂，β̂ = atan(κ_exec·ref_ahead)（κ_exec = 上一拍指令
        # 曲率，ω = v·κ_exec 的执行即后轴曲率）。推导见 __init__ 的 ref_ahead。
        # 前进：速度方向 = 航向 + β̂；倒车（虚拟车头）：速度方向 = 虚拟航向 − β̂。
        # 航向项反馈 φ_v 才是真正决定 de/ds 的量；纯跟踪的 α 也减去同一项。
        beta_hat = math.atan(self._kappa_exec * self.ref_ahead)
        g_sg = 1.0 if gear > 0 else -1.0
        phi_v = wrap(phi + g_sg * beta_hat)
        # ---------------- 预瞄纯跟踪 κ_pp（仅 preview>0 时启用）----------------
        # κ_pp = 2·sin(α)/L_p，α =「车头指向」到「垂足前方 L_p 米处路径点」的夹角。
        # 这是纯跟踪的标准形式：横向偏差和航向偏差自然合并在 α 里，而且是**几何
        # 无差拍**的 —— 只要 L_p 不太短，闭环对定位噪声和执行滞后都是稳定的。
        # 相比下面的 e/φ 反馈式（κ = κ_ref − k_e·e − k_φ·φ），它的等效航向增益是
        # 2/L_p ≈ 0.9（L_p=2.2 m），而反馈式的 k_head=1.8 是它的两倍：一旦航向
        # 被噪声带偏 6°，反馈式直接把 κ 顶到限幅，车就被迫摆回来，于是左右摇。
        k_pp = None
        la_d = 0.0
        la_a = 0.0
        # [同学 C 修复] 预瞄距离在路线末段必须跟着剩余距离收小。
        # 巡航的 2.8 m 预瞄对着「最后一个 90° 拐进车道节点」是无解的：纯跟踪
        # 必然切角，实测交接位姿比路线终点偏 1.0~1.4 m、航向差 13°。
        # 而离线在**正好的车道节点**上规划只得到 9.1 m / 0 换挡的干净解；
        # 位姿一偏就变成 14.7 m / 1 换挡的长 S 形，跟车器跟不动
        # （横向偏差一路涨到 1.2 m，然后被「横向偏差持续」掐断重规划）。
        # 所以最后 6 m 把预瞄按 remaining/6 线性收到 1.0 m。
        # 入位段（tail_anchor=False）不收缩：0.45 m/s 慢速下 Ld 收到 1.0 m
        # 会把定位横向残差按 2/L_d 放大（κ 在 ±限幅乱撞），宁可全程 2.4 m。
        pv = self.preview
        if self.tail_anchor and pv > 0.0 and remaining < 6.0:
            pv = max(1.0, pv * remaining / 6.0)
        if pv > 0.0:
            la = self._lookahead(seg_i, seg_t, pv,
                                 anchor_end=self.tail_anchor)
            if la is not None:
                lx, ly, _ = la
                dxl, dyl = lx - x, ly - y
                Ld = math.hypot(dxl, dyl)
                la_d = Ld
                if Ld > 0.30:
                    alpha = wrap(math.atan2(dyl, dxl) - th)
                    if gear < 0:
                        # 倒车 = 车头转 180° 后前进，α 整体减 π
                        alpha = wrap(alpha - math.pi)
                    # [同学 C 本轮 §6.28] α 从**速度方向**起算（减 gear·β̂）：
                    # 纯跟踪的几何推导假设参考点沿「瞄准方向」运动，而底盘
                    # 原点的真实速度偏航向 β̂。不补的话弧上 α 恒差一个 β̂，
                    # 闭环把航向压到切线、速度却斜着走（e 单调发散）。
                    alpha = wrap(alpha - g_sg * beta_hat)
                    la_a = alpha
                    # 分母下限 ld_floor：锚定在终点上时 Ld 随车逼近塌缩，
                    # 2/L_d 增益发散会把定位横向残差放大成顶满的 κ。(floor=0
                    # 时退化为旧行为。)
                    k_pp = 2.0 * math.sin(alpha) / max(Ld, self.ld_floor)
        # 跟踪律（曲率前馈 + 横向/航向反馈）：
        #     前进：κ = κ_ref − k_e·e − k_φ·φ
        #     倒车：κ = κ_ref + k_e·e − k_φ·φ
        # 前馈项**不随档位变号**：这里的 ω 是「期望偏航率」，gz 的
        # AckermannSteering 用 atan(ω·L/v)（单参数 atan），v<0 时负负得正，
        # 所以倒车下发的 ω 会被原样复现，路径要求多少就下发多少。
        # 横向反馈项要变号：同一个「车偏在路径左侧」，前进要往右打、倒车要往左打，
        # 因为倒车时速度方向与车头相反。航向项则不变号（它修的是速度方向偏差）。
        # 旧代码把 φ 写成 wrap(th + π − thr_p)：对倒车段恒等于 ±π（wrap 边界上
        # 逐帧跳正负），k_φ·φ ≈ ±3.77 直接把曲率顶到限幅，ω 被钉在 ±0.09 来回跳，
        # 倒车的横向/航向反馈等于完全失效（开环倒车），实测 5 s 内横向偏差
        # 从 0.00 涨到 0.66 m、终点 2.5 m / 24°。
        if k_pp is not None and not self._la_flip:
            # 预瞄纯跟踪：直接用几何曲率，不再叠加 e/φ 反馈（两者叠加等于又把
            # 等效航向增益抬回 1.8 以上，摆动会重新出现）。
            # [同学 C 本轮] `self._la_flip`：预瞄窗口跨**曲率变号点**时 k_pp
            # 不可信 —— S 形两段弧的弦把曲率平均掉（E2 收尾弧入口 k_pp=+0.03
            # vs 段需求 -0.165），φ 以 4°/s 攀 12°、3 m 收尾弧顶死限幅追不回。
            # 弃用 k_pp 落到下面的降级律（前馈 k_ref + 航向反馈），在前馈
            # 正确的前提下 φ 以 τ=1/(|v|·k_head)≈2.4 s 指数收敛。
            curvature = k_pp
            k_fb = 0.0
        elif self.preview > 0.0 and not self.tail_anchor:
            # [同学 C 本轮] 入位段的**预瞄降级律：航向优先**。两种触发：
            # ① 预瞄跨不过换挡点（_lookahead 对 gear_cut 返回 None）；
            # ② 预瞄窗口跨曲率变号点（self._la_flip，见纯跟踪分支的说明）。
            # 退回反馈但丢掉横向项：反馈律的稳态是 φ = -k_lat·e/k_head ——
            # 拿 5° 的欠转去换 0.1 m 的横向偏差。换挡后的弧普遍只剩 ~20%
            # 曲率余量（E2 倒车弧 κ=0.165 vs 执行限幅 0.202），追不回这个
            # 欠转：seed5 实测（OU 漂移 0.15 m 噪声下）换挡点前 e_sig≈-0.15、
            # φ 被稳态顶到 +5.7°（真实 +10°），倒车弧 κ 全程顶死、φ 花 14 s
            # 才衰减完，期间横向偏差积分到 -0.53 m，末段航向残差 22° 且剐蹭。
            # 宁可带着小 e 过换挡点（下一段的纯跟踪自己会收敛 e），
            # 把 φ 交给 k_head 压到 0。
            # 只对入位段（tail_anchor=False）生效：路线段的 None 窗口只有
            # 末尾 0.5 m，其 e 项还承担着交接位姿的对准，保持原律。
            # [同学 C 本轮 §6.28] 航向项用速度方向误差 φ_v（β 补偿）。
            curvature = k_ref
            k_fb = -self.k_head * phi_v
        elif gear > 0:
            curvature = k_ref
            k_fb = -self.k_lat * e_lat - self.k_head * phi_v
        else:
            curvature = k_ref
            # 倒车段：横向偏差的 P 与 I 同号（见下方反馈符号说明），
            # 积分项用来抵消常值曲率偏差（见 __init__ 里的推导）。
            # [同学 C 本轮 §6.28] 航向项用速度方向误差 φ_v（β 补偿）——
            # W7「倒车弧同心圆、φ→0 但 e 线性涨」的根因正是这一项没补 β。
            if self.k_int > 0.0:
                self._e_int = max(-self.int_limit, min(self.int_limit,
                                                      self._e_int + e_lat * dt))
            else:
                self._e_int = 0.0
            k_fb = (self.k_lat * e_lat + self.k_int * self._e_int
                    - self.k_head * phi_v)

        # [同学 C 修复] `steer_gain` 的语义修正：它是**执行器偏航率不足的补偿**，
        # 必须乘在**整条指令**上（前馈 + 反馈），不能只乘反馈。
        # 真机标定：车辆实际偏航率只有指令的 0.65（`UNDER`）。要让实际曲率等于
        # 参考曲率，指令就得是 `κ_ref / 0.65 = 1.54·κ_ref`，所以 `steer_gain`
        # 取 1.55（原来写 1.40，补偿量不够，倒车弧上永远差 9%）。
        # 前提是**指令不许顶到限幅** —— 一旦顶住，实际只给出
        # `0.65 × κ_max`，反馈就完全失去作用（真机 W7 的 κ 长期 railed 在
        # ±0.202，横向偏差在 9 m 弧上从 0.06 m 涨到 1.05 m）。
        # 所以规划侧用的是「大半径圆弧」：`1.55·κ_ref` 只占到限幅的 ~75%，
        # 留出 25% 给反馈（见 `config/staging_poses.yaml` 与 compute_staging.py）。
        kappa_max = math.tan(self.max_steer) / self.wheelbase
        curvature = (curvature + k_fb) * self.steer_gain
        curvature = max(-kappa_max, min(kappa_max, curvature))
        self._lat = abs(e_lat)
        # [同学 C 本轮 §6.28] 记录本拍指令曲率，供下一拍的 β̂ = atan(κ·ref_ahead)。
        # 放在阻尼项之前：阻尼修的是偏航率瞬态，β 补偿要的是「车正在执行的
        # 稳态曲率」，取限幅后的主指令即可（执行链 4% 欠转折成 β̂ 误差 <0.5°）。
        kappa_sent = curvature
        # 诊断量（只读，不进控制）：换挡/倒车段出问题时可一眼看出是参考点错位
        # （thr_p 与 th 差几十度）还是增益饱和（κ 贴限幅）
        self.dbg = {'thr': thr_p, 'phi': phi, 'e': e_lat, 'kappa': curvature,
                    'k_ref': k_ref, 'seg': self._proj_seg, 't': self._proj_t,
                    'gear': gear, 'k_pp': k_pp, 'la_d': la_d, 'la_a': la_a,
                    'flip': self._la_flip, 'beta': beta_hat, 'phi_v': phi_v}

        # 速度规划：曲率、剩余距离、风险系数
        # 末段（<2 m）再压一档速度：实测真实执行器有相位滞后，0.40 m/s 进最后
        # 一段圆弧时横向偏差会在 1.5 m 内从 0.19 m 涨到 0.39 m，终点误差被带到
        # 0.62 m（离线理想执行器只有 0.15 m）。慢下来等于给反馈更多修正机会。
        v_cap = self.v_max
        if remaining < 2.0:
            v_cap = min(v_cap, 0.40)
        if remaining < 1.5:
            v_cap = min(v_cap, 0.28)
        v = v_cap
        if abs(curvature) > 1e-3:
            v = min(v, math.sqrt(self.a_lat / abs(curvature)))
        v = min(v, math.sqrt(max(0.0, 2.0 * self.a_dec * max(0.05, remaining - 0.15))))
        v = max(0.0, min(v, v_cap)) * max(0.0, speed_scale)
        if 0.0 < v < self.v_min and remaining > 0.6:
            v = self.v_min

        # 偏航率阻尼：把「实测偏航率 − 期望偏航率」反馈进曲率。
        #
        # [同学 C 本轮] 两条改动：
        # 1. **前进/倒车都启用**。原来写死 `gear > 0`，理由是「倒车段已验证可用」——
        #    但那条验证是在没有真实偏航率信号的情况下做的（位姿差分恒 0，等于没开）。
        #    现在偏航率来自 IMU（100 Hz），倒车段的航向环同样是「延迟 + 高增益」，
        #    一样需要微分项。倒车的期望偏航率是 `|v|·κ`（不是 `v·κ`，见下面 omega 的
        #    推导），所以分母用 `|v|`。
        # 2. 期望值用**限幅前**的指令 `w_ref = (前进 v·κ / 倒车 |v|·κ)`，
        #    而不是旧的 `v·k_ref`：旧写法的期望值里没有反馈项，等于用「参考曲率」
        #    去比「实测偏航率」，误差里混进了本该由反馈消掉的那部分。
        if (yaw_rate is not None and self.k_rate > 0.0 and abs(v) > 0.15):
            vv = v if gear > 0 else abs(v)
            w_ref = vv * curvature
            curvature -= self.k_rate * (yaw_rate - w_ref) / vv
            curvature = max(-kappa_max, min(kappa_max, curvature))
            kappa_sent = curvature
        # β 补偿的执行曲率估计（ω = v·κ_sent 即真实偏航率，见 §6.28）
        self._kappa_exec = kappa_sent

        # 偏航角速度：前进 ω = v·κ；倒车用虚拟车头推导，ω = |v|·κ
        # （steer_gain 已在上面并入 curvature，这里不能再乘一次）
        speed = v * (1.0 if gear > 0 else -1.0)
        omega = v * curvature
        if gear < 0:
            omega = abs(omega) * (1.0 if curvature > 0 else -1.0)
        # 角速度兜底：κ 已被 kappa_max 限死，这里只是数值保护
        omega_lim = abs(v) * kappa_max
        omega = max(-omega_lim, min(omega_lim, omega))

        # [同学 C] 指令一阶低通：定位残差（map→base 相对真值峰峰值 1.41 m）里
        # 的高频成分会在这一帧变成一次打舵，下一帧又反着来 —— 频率正好落在
        # 车身横摆的共振带里（实测周期 4.5 s 的摆动叠加了 ~2 Hz 的抖）。
        # 低通把「噪声 → 舵」的通道衰减掉，而对 0.2 Hz 级别的真实所需修正几乎
        # 无损（τ=0.2 s 时 0.2 Hz 处衰减仅 2%）。放在速率限幅**之前**，
        # 两级一起把 ω 压成一条平滑曲线。
        if self.smooth > 0.0:
            a = max(1e-3, min(1.0, dt / (self.smooth + dt)))
            if self._omega_lp is None:
                self._omega_lp = omega
            else:
                self._omega_lp = (1.0 - a) * self._omega_lp + a * omega
            omega = self._omega_lp
        else:
            self._omega_lp = omega

        # [同学 C] 转向速率限制 —— 修「主干道左右摇摆」的关键一层。
        # 实测：定位（map→base）相对真值的横向残差峰峰值 1.41 m、60 s 内符号
        # 翻转 8 次；而这一层在 15 cm 误差时就饱和，于是 ω 被钉在 ±0.283
        # （= 1.4 m/s × κ_max）来回跳，真值偏航率 ±0.44 rad/s、翻转 14 次，
        # 车就真的左右摆起来了。
        # 这里给方向盘加一个速率上限：定位再怎么跳，舵也只能匀速打过去，
        # 摆幅立刻收敛成一条平滑的修正曲线。代价是极端机动会慢一点。
        if self.steer_rate > 0.0 and abs(v) > 1e-3:
            step = self.steer_rate * max(dt, 1e-3)
            if self._last_omega is not None:
                omega = max(self._last_omega - step,
                            min(self._last_omega + step, omega))
        self._last_omega = omega

        return FollowCommand(speed, omega, False, self.idx, remaining, self._lat,
                            yaw_end, v, gear, stuck)
