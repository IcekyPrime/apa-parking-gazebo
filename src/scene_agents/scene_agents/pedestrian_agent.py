#!/usr/bin/env python3
"""行人驱动：沿**闭环**路径行走，开局随机落点，遇到车辆自动停步。

为什么要自己驱动，而不是用 Gazebo 的 TrajectoryFollower
--------------------------------------------------------
TrajectoryFollower 靠给连杆施加恒定推力追航点：不会提前减速（实测冲过航点
4~16 m 再回头），而行人是圆柱碰撞体、没有轮子，一旦被推上/推下人行道那种
0.12 m 台阶就永久卡死 —— 画面里就是「行人一动不动」。

这里改成运动学搬运：直接调 gz 的 `/world/<world>/set_pose` 服务
（ROS 侧 `ros_gz_interfaces/srv/SetEntityPose`），按固定速度沿路径推进。
模型保持 `<static>true</static>`（不受重力、不会被车推走），但**保留碰撞体**，
所以 2D 激光照样扫得到，动态避障测试仍然有效。

相对上一版的三处改动
--------------------
1. **闭环**：路径首尾自动闭合，走完一圈接着走，不再「到头折返」。
   上一版的往返轨迹会让行人在返回时直接穿过来车 —— 用户实测到的问题。
2. **随机起点**：每次仿真开始，每个行人在自己闭环上的随机弧长处出现。
3. **遇车停步**：`stop_radius` 内有任意车辆就停住（不推进弧长），车走开再继续。

参数
----
routes           : "name=x1,y1;x2,y2;...|name2=..."  每个行人一条**闭环**路径
speed            : 行走速度 (m/s)
evade_clear      : 让行半径 r (m)，**只在前方扇形内**有效（见 fov）
fov              : 让行扇形全角（度），默认 140°（<180°）
wait_before_back : 被挡后先原地等多久 (s)
back_dist        : 一次让行沿原路径后退多远 (m)
back_speed       : 后退速度 (m/s)
hold_after_back  : 退让之后**第二次等待**的时长 (s)，强制 ≥10 s
vehicles         : 参与判定车辆 odom 话题，逗号分隔
seed             : 随机种子，0 表示用系统时间

[同学 C 修复] 为什么是扇形而不是圆
--------------------------------
旧判据是「车体矩形距离 < r 就停」，等于全向视觉：车从身边侧向 1.5 m 开过、
甚至已经开到身后，行人也站着不动。用户实测到的现象是「行人明明可以走开却
停住，车只好反复调整/重新规划」，两边互相等。真实的人眼视野不到 180°，只关心
**自己前进方向上**的障碍，所以这里把 r 改成 ±fov/2 的扇形，用射线与车体矩形
求交，取扇形内的最近距离。

[同学 C 修复] 被挡之后不许「蠕动」
-------------------------------
旧逻辑是「每帧被挡就退一小步、挡一解除就往前走一步」，人贴在保险杠前会一格格
地来回试探（用户截图里的「堵车」），车也走不了。现在交给 `YieldPolicy`：
**先等 wait_before_back 秒 → 仍被挡就沿原路径主动后退 back_dist 米 →
再站住 hold_after_back（≥10 s）→ 然后才重新尝试前进**。
"""

import math
import random

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import Odometry
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose


class YieldPolicy:
    """被挡时行人的动作序列：**等待 → 主动退让 → 再等待 ≥10 s → 再尝试前进**。

    为什么要有状态机
    ----------------
    旧实现是「每帧被挡就退一小步、挡一解除就往前走一步」，等价于一个没有迟滞的
    比例控制器：人被挡在车前时会在原地一格格地**蠕动**（用户截图里行人贴着保险杠
    反复试探），而车一直过不去，双方耗在原地。实测一次 480 s 运行里行人把 15 m
    后退预算磨光、自车被激光护栏冻结 300 s，最后「全局路线行驶超时，放弃」。

    现在换成显式的三段，每段都有明确的门限：

    1. ``WAIT``：先原地等 ``wait_before_back`` 秒。多数情况车马上就走，
       不必真的让开；
    2. ``BACK``：还挡着 → 沿自己的闭环**主动后退** ``back_dist`` 米。
       这里是「真让开」，不是一格一格地蹭；
    3. ``HOLD``：退完再**至少站住** ``hold_after_back`` 秒（默认 10.5 s）。
       这一段时间里即使障碍已经消失也**不提前走** —— 目的就是给车足够时间
       把这段路走完，否则人一恢复行走又立刻堵回去，等于没让；
    4. 之后再回到 ``WALK`` 重新尝试前进。

    ``sample()`` 返回**带符号的弧长增量**（>0 前进、<0 后退、0 站立），
    由调用方推进 `LoopWalker`，这样策略本身是纯逻辑、可以离线单测。
    """

    WALK = 'walk'
    WAIT = 'wait'
    BACK = 'back'
    HOLD = 'hold'

    def __init__(self, walk_speed=1.1, back_speed=0.6,
                 wait_before_back=2.5, back_dist=2.0, hold_after_back=10.5):
        self.walk_speed = float(walk_speed)
        self.back_speed = float(back_speed)
        self.wait_before_back = float(wait_before_back)
        self.back_dist = float(back_dist)
        self.hold_after_back = max(10.0, float(hold_after_back))
        self.mode = self.WALK
        self.timer = 0.0          # 当前阶段已持续的时间
        self.back_left = 0.0      # BACK 阶段还要退多少米
        self.cycles = 0           # 本次阻塞已经让过几轮（只用于日志）

    def reset(self):
        self.mode = self.WALK
        self.timer = 0.0
        self.back_left = 0.0
        self.cycles = 0

    def abort_back(self):
        """后退方向不安全（身后就是车）时中止 BACK，直接进入 HOLD 站住。"""
        self.mode = self.HOLD
        self.timer = 0.0
        self.back_left = 0.0

    def sample(self, blocked, dt, budget_left=1e9):
        """推进一个控制周期，返回本次的弧长增量（m，带符号）。"""
        if self.mode == self.WALK:
            if not blocked:
                return self.walk_speed * dt
            self.mode = self.WAIT
            self.timer = 0.0
            return 0.0

        if self.mode == self.WAIT:
            self.timer += dt
            if not blocked:                       # 障碍自己走了，立刻继续
                self.mode = self.WALK
                self.timer = 0.0
                return self.walk_speed * dt
            if self.timer >= self.wait_before_back:
                self.cycles += 1
                if budget_left <= 0.0:            # 预算用完：不再退，只能等
                    self.mode = self.HOLD
                    self.timer = 0.0
                else:
                    self.mode = self.BACK
                    self.back_left = self.back_dist
                    self.timer = 0.0
            return 0.0

        if self.mode == self.BACK:
            if self.back_left <= 1e-6 or budget_left <= 0.0:
                self.mode = self.HOLD
                self.timer = 0.0
                self.back_left = 0.0
                return 0.0
            ds = min(self.back_speed * dt, self.back_left)
            self.back_left -= ds
            return -ds

        # HOLD：**不因为障碍消失而提前结束**，必须站满 hold_after_back
        self.timer += dt
        if self.timer >= self.hold_after_back:
            self.mode = self.WALK
            self.timer = 0.0
            return self.walk_speed * dt
        return 0.0


class LoopWalker:
    """沿闭环折线行走（弧长参数化，首尾相连）。"""

    def __init__(self, name, pts):
        self.name = name
        self.pts = list(pts)
        self.seg_len = [math.hypot(self.pts[(i + 1) % len(self.pts)][0] - self.pts[i][0],
                                   self.pts[(i + 1) % len(self.pts)][1] - self.pts[i][1])
                        for i in range(len(self.pts))]
        self.total = sum(self.seg_len)
        self.s = 0.0
        self.stopped = False
        self.back_used = 0.0      # 本次阻塞累计后退米数（预算用）
        self.walk_clear = 0.0     # 连续正常前进米数（够长就复位预算）
        self.static_t = 0.0       # 挡路的车连续静止的时长（超过 12 s 恢复预算）
        self.side = 0.0           # 侧向让开量（m，路径左正）
        self.side_target = 0.0    # 侧向让开目标
        # [同学 C 本轮] 「被挡」的**迟滞闩锁**。没有它会出两个问题：
        #   ① 人的路径是弯的，yaw 一转车就掉出 ±70° 扇形 → `d_path` 变 inf →
        #      立刻退出 WAIT → 再进来，日志每 0.2 s 刷一对「被挡 / 重新起步前进」，
        #      2.5 s 的等待永远等不满，后面的「退让 / 侧移」根本不执行；
        #   ② 人一边被挡一边蹭着往前走（WALK 每拍推进 0.02 m），看起来就是「抽搐」。
        # 闩锁规则：`d_path < evade_clear` 立即置位；只有 `d_path > resume_radius`
        # **持续 0.6 s** 才解锁。参数 `resume_radius` 本来就是为此声明的，
        # 但之前只用在扇形 reach 上，没有真正做迟滞。
        self.block_latch = False
        self.clear_t = 0.0
        self.policy = None        # YieldPolicy，由 PedestrianAgent 注入
        self._log_mode = YieldPolicy.WALK   # 上一次打日志时的状态（只在变化时打）

    def set_s(self, s):
        self.s = s % self.total if self.total > 1e-6 else 0.0

    def advance(self, ds):
        """推进 ds 米，返回 (x, y, yaw)。"""
        if self.total < 1e-6:
            return self.pts[0][0], self.pts[0][1], 0.0
        self.s = (self.s + ds) % self.total
        return self.at(self.s)

    def at(self, s):
        acc = 0.0
        n = len(self.pts)
        for i, ln in enumerate(self.seg_len):
            if s <= acc + ln or i == n - 1:
                u = (s - acc) / ln if ln > 1e-9 else 0.0
                p0, p1 = self.pts[i], self.pts[(i + 1) % n]
                x = p0[0] + (p1[0] - p0[0]) * u
                y = p0[1] + (p1[1] - p0[1]) * u
                yaw = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
                return x, y, yaw
            acc += ln
        p = self.pts[-1]
        return p[0], p[1], 0.0


def parse_routes(text):
    out = []
    for item in text.split('|'):
        item = item.strip()
        if not item or '=' not in item:
            continue
        name, body = item.split('=', 1)
        pts = []
        for seg in body.split(';'):
            seg = seg.strip()
            if not seg:
                continue
            xy = seg.split(',')
            if len(xy) < 2:
                continue
            pts.append((float(xy[0]), float(xy[1])))
        if len(pts) >= 3:
            out.append(LoopWalker(name.strip(), pts))
    return out


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class PedestrianAgent(Node):
    def __init__(self):
        super().__init__('pedestrian_agent')
        self.declare_parameter('world', 'city_parking_lot')
        self.declare_parameter('routes', '')
        self.declare_parameter('speed', 1.1)
        self.declare_parameter('rate', 10.0)
        self.declare_parameter('z', 0.05)
        # 让行滞后：进 evade_clear 就停/退，扇形内退到 resume_radius 以外才继续。
        # 注意这两个值是**离车体外廓**的距离（不是到车辆中心的距离，旧版用中心距
        # 3.5 m，对 4.35 m 长的车只等于离车头 1.3 m，视觉上人就是贴着车走）。
        # 没有滞后的话，车一动行人就恢复、车一停行人又堵上来，会和车的揉库
        # 动作互相追着跑（用户实测到「车往前人也往前，车想倒车时被人挡住」）。
        self.declare_parameter('resume_radius', 4.5)
        # 让行扇形的**全角**（度）。<180° 才符合「人眼只看前方」的直觉。
        # 180° 等价于旧的圆判据，>180° 会连身后的车也拦。
        self.declare_parameter('fov', 140.0)
        # 「车在动」的判据（m/s）。仅用于日志/诊断：现在即使车停着，行人也会
        # 按「等待→退让→再等」把路让开，不再靠「忽略停着的车」来绕开互锁。
        self.declare_parameter('move_speed', 0.25)
        # 车体矩形（半长/半宽）+ 越过它的余量。任何情况下都不许进入该矩形。
        self.declare_parameter('body_half_len', 2.20)
        self.declare_parameter('body_half_wid', 1.00)
        self.declare_parameter('body_margin', 0.35)
        # 让行判据：前方扇形内离车体外廓小于该值就算「被挡」。
        # [同学 C] 1.6 → 3.0：车的让行距离是 `ped_stop = 7.0 m`，人这边如果只有
        # 1.6 m，就会落进「车觉得被堵、人觉得自己没挡路」的空档里，谁都不动
        # （用户截图里蓝车和蓝色行人卡死）。人的触发距离必须明显大于车的，
        # 才能在人被车「等到」之前就开始让。3.0 m 是折中：再大人会在路口
        # 被每一辆路过的车拦住，反而挡住自车。
        self.declare_parameter('evade_clear', 3.0)
        # 侧向让开：被挡时朝背离那台车的一侧横移这么多米。必须 > 车的走廊半宽
        # （`corridor_half = 2.4 m`），否则横移之后人仍然在车的行进带子里。
        self.declare_parameter('side_step', 2.8)
        self.declare_parameter('side_speed', 0.6)
        # 行人要让到「离车体轴线」这么远才算走出车的行进带子。
        # 车的走廊半宽是 2.4 m（`path_agent.corridor_half`），加人的身体半径 0.3
        # 和 0.2 m 余量 = 2.9 m。侧移目标按这个值反算，不再固定 2.8 m。
        self.declare_parameter('corridor_clear', 2.9)
        # 本次阻塞累计后退预算（m）：用完就只站不退了，避免无限倒着绕圈。
        self.declare_parameter('evade_max', 15.0)
        # ---------------- 被挡后的动作序列（见 YieldPolicy）----------------
        # 先原地等这么久；还挡着才真的退让。多数情况车马上就走，人不必让。
        self.declare_parameter('wait_before_back', 2.5)
        # 一次让行退多远（m）。2 m 足够把人的身体和车宽错开，又不会退进另一条道。
        self.declare_parameter('back_dist', 2.0)
        # 后退速度（m/s）：比正常走路慢，画面上像「侧身后撤」而不是倒着跑。
        self.declare_parameter('back_speed', 0.6)
        # 退让之后的**第二次等待**（s）。要求「尽可能达到 10 s 以上」，构造函数里
        # 还会再夹一层 max(10.0, ...)，参数写小了也压不下去。
        self.declare_parameter('hold_after_back', 10.5)
        # 扇形里射线数。全角 140°、17 条射线 → 角步 8.75°，2 m 处相邻射线间距
        # 只有 0.31 m，比车身宽度小得多，不会从两条射线之间漏掉整台车。
        self.declare_parameter('fov_rays', 17)
        self.declare_parameter('vehicles', '/odom_gt,/traffic/odom_gt,/traffic2/odom_gt')
        self.declare_parameter('seed', 0)

        world = self.get_parameter('world').value
        self.speed = float(self.get_parameter('speed').value)
        self.z = float(self.get_parameter('z').value)
        self.resume_r = float(self.get_parameter('resume_radius').value)
        self.fov_half = math.radians(
            max(10.0, min(180.0, float(self.get_parameter('fov').value)))) * 0.5
        self.fov_rays = max(3, int(self.get_parameter('fov_rays').value))
        self.move_th = float(self.get_parameter('move_speed').value)
        self.body_hl = float(self.get_parameter('body_half_len').value)
        self.body_hw = float(self.get_parameter('body_half_wid').value)
        self.body_m = float(self.get_parameter('body_margin').value)
        self.evade_clear = float(self.get_parameter('evade_clear').value)
        self.side_step = float(self.get_parameter('side_step').value)
        self.cor_clear = float(self.get_parameter('corridor_clear').value)
        self.side_speed = float(self.get_parameter('side_speed').value)
        self.evade_max = float(self.get_parameter('evade_max').value)
        self.policy_kw = dict(
            walk_speed=self.speed,
            back_speed=float(self.get_parameter('back_speed').value),
            wait_before_back=float(self.get_parameter('wait_before_back').value),
            back_dist=float(self.get_parameter('back_dist').value),
            hold_after_back=float(self.get_parameter('hold_after_back').value))
        seed = int(self.get_parameter('seed').value)
        self.rng = random.Random(seed if seed else None)

        self.walkers = parse_routes(self.get_parameter('routes').value)
        # 每个行人一套让行状态机（等待 → 退让 → 再等待 → 再尝试）
        for w in self.walkers:
            w.policy = YieldPolicy(**self.policy_kw)
        # 随机落点：每个行人在自己闭环的随机弧长处出现
        for w in self.walkers:
            w.set_s(self.rng.uniform(0.0, max(w.total, 1e-3)))

        self.svc = f'/world/{world}/set_pose'
        self.cli = self.create_client(SetEntityPose, self.svc)

        # 行人位姿发布：干扰车（path_agent）拿它做让行判据。
        # 没有这条话题时，干扰车只认识「车」，会直接从行人身上压过去 ——
        # 实测行人被压到「离车体 0.0 m」并反复触发兜底穿模。
        self.pub_states = self.create_publisher(PoseArray, '/pedestrians', 10)

        self.vehicles = {}
        for topic in str(self.get_parameter('vehicles').value).split(','):
            topic = topic.strip()
            if topic:
                self.create_subscription(
                    Odometry, topic, lambda m, t=topic: self.on_odom(m, t), 20)
                self.vehicles[topic] = None

        self.dt = 1.0 / float(self.get_parameter('rate').value)
        self._warned = False
        self.create_timer(self.dt, self.tick)

        pol = self.walkers[0].policy if self.walkers else YieldPolicy()
        self.get_logger().info(
            f'行人驱动已启动：{len(self.walkers)} 组，速度 {self.speed:.1f} m/s，'
            f'让行扇形 {math.degrees(self.fov_half) * 2:.0f}°×{self.evade_clear:.1f} m，'
            f'被挡后：等 {pol.wait_before_back:.1f} s → 退 {pol.back_dist:.1f} m'
            f'（{pol.back_speed:.1f} m/s）→ 再等 {pol.hold_after_back:.1f} s → 重新前进，'
            f'监视车辆 {list(self.vehicles)}')
        for w in self.walkers:
            self.get_logger().info(
                f'  {w.name}: {len(w.pts)} 个航点，闭环长 {w.total:.0f} m，'
                f'开局位于弧长 {w.s:.0f} m 处')

    def on_odom(self, msg, topic):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        now = self.get_clock().now().nanoseconds * 1e-9
        prev = self.vehicles.get(topic)
        spd = 0.0
        if prev is not None and now - prev[3] > 1e-3:
            raw = math.hypot(p.x - prev[0], p.y - prev[1]) / (now - prev[3])
            # EMA 平滑，避免单帧定位跳变被当成「车在动」
            spd = 0.7 * prev[4] + 0.3 * raw
        self.vehicles[topic] = (p.x, p.y, yaw, now, spd)

    def _body_dist(self, x, y):
        """行人对**车体矩形**的距离（m）：车身内为 0，车外为到矩形边界的最短距离。

        [同学 C 修复] 旧判据用的是「到车辆 odom（后轴中心）的距离 < 3.5 m」。
        对一台 4.35 m 长、1.85 m 宽的车来说，3.5 m 中心距只相当于**离车头保险杠
        1.3 m**、离侧面 2.6 m —— 视觉上就是「人贴在车头上」，用户看到的现象是
        「避让根本没生效」。改成按车体外廓算距离后，让行半径才有直观意义。
        """
        best = float('inf')
        for v in self.vehicles.values():
            if v is None:
                continue
            dx, dy = x - v[0], y - v[1]
            c, s = math.cos(v[2]), math.sin(v[2])
            lon = c * dx + s * dy
            lat = -s * dx + c * dy
            ox = max(0.0, abs(lon) - self.body_hl)
            oy = max(0.0, abs(lat) - self.body_hw)
            best = min(best, math.hypot(ox, oy))
        return best

    def _nearest_vehicle(self, x, y):
        """最近车辆的 (距离, 是否在走动)，仅用于日志。"""
        best = (float('inf'), False)
        for v in self.vehicles.values():
            if v is None:
                continue
            d = math.hypot(v[0] - x, v[1] - y)
            if d < best[0]:
                best = (d, v[4] > 0.2)
        return best

    def _nearest_vehicle_pose(self, x, y):
        """最近车辆的 (x, y, yaw)；没有车返回 None。侧向让开要用它的方位。"""
        best, out = float('inf'), None
        for v in self.vehicles.values():
            if v is None:
                continue
            d = math.hypot(v[0] - x, v[1] - y)
            if d < best:
                best, out = d, (v[0], v[1], v[2])
        return out

    @staticmethod
    def _ray_box(ox, oy, dx, dy, hl, hw, reach):
        """射线与「轴对齐矩形」求交（调用前已把射线原点/方向变换到矩形坐标系）。

        标准的 slab 法：对 x、y 两个方向各求一次参数区间，两个区间的交集就是
        射线穿过矩形的参数区间 [tmin, tmax]。原点已在矩形内时 tmin 保持 0。
        返回最近的正交点距离（m），不相交返回 None。
        """
        tmin, tmax = 0.0, reach
        for o, d, half in ((ox, dx, hl), (oy, dy, hw)):
            if abs(d) < 1e-9:
                if abs(o) > half:
                    return None
                continue
            t1 = (-half - o) / d
            t2 = (half - o) / d
            if t1 > t2:
                t1, t2 = t2, t1
            if t1 > tmin:
                tmin = t1
            if t2 < tmax:
                tmax = t2
            if tmin > tmax:
                return None
        return tmin if tmax >= 0.0 else None

    def _sector_hit(self, x, y, yaw, reach):
        """行人**前方扇形**内的最近车体距离 + 那台车是否在行驶 → (dist, moving)。

        [同学 C 修复] 这就是用户提的「把半径 r 改成扇形」：以行人当前行进方向为
        轴，张角 ±fov/2（合计 <180°），用 fov_rays 条射线与每台车的车体矩形求交，
        取所有交点的最小距离。落在扇形外的车（侧向擦过、已经开到身后）**不参与**
        判定，行人照常走 —— 旧的全向半径判据会让行人在这里白站十几秒，车也跟着
        重新规划。

        同时返回「挡路那台车是否在动」：**已经停住的车是对我让行，不是我该让它**，
        行人要继续走（见 tick 里的判据）。否则会出现「车停在人前面等人过、人又停下
        让车」的双向等待，实测把整场仿真锁死 300 s。
        """
        best = float('inf')
        best_moving = False
        n = self.fov_rays
        span = 2.0 * self.fov_half
        for v in self.vehicles.values():
            if v is None:
                continue
            dx, dy = x - v[0], y - v[1]
            c, s = math.cos(v[2]), math.sin(v[2])
            # 行人位置换算到车辆坐标系（车辆为轴对齐矩形）
            ox = c * dx + s * dy
            oy = -s * dx + c * dy
            for k in range(n):
                ang = yaw - self.fov_half + span * k / (n - 1)
                wx, wy = math.cos(ang), math.sin(ang)
                # 世界方向 -> 车辆坐标系（只旋转，长度不变，所以 t 就是米）
                rx = c * wx + s * wy
                ry = -s * wx + c * wy
                t = self._ray_box(ox, oy, rx, ry, self.body_hl, self.body_hw, reach)
                if t is not None and t < best:
                    best = t
                    best_moving = v[4] > self.move_th
        return best, best_moving

    def _inside_body(self, x, y):
        """是否已经贴到车体（离车体外廓小于 body_margin）。

        这是**全向**的安全兜底，不参与扇形判断：行人可以决定「不在我路上 → 我走」，
        但任何方向都不许走进车身。
        """
        return self._body_dist(x, y) < self.body_m

    def tick(self):
        if not self.cli.service_is_ready():
            if not self._warned:
                self._warned = True
                self.get_logger().info(f'等待 Gazebo 服务 {self.svc} …')
            return

        poses = []
        for w in self.walkers:
            cx, cy, cyaw = w.at(w.s)
            # [同学 C 修复] 两个判据分工明确：
            #   · d_path —— **前方扇形**内的最近车体距离，是「要不要让」的唯一依据；
            #   · d_body —— 全向车体距离，只做「绝不许走进车身」的安全兜底。
            # 旧版用 d_body < evade_clear 当让行依据，等于全向视觉：车在侧面
            # 1.5 m 擦过、甚至已经开到身后，行人也站着不动，于是「人明明能走开
            # 却停住，车只好重新规划」。用户要的就是这条改成扇形。
            d_body = self._body_dist(cx, cy)
            reach = max(self.evade_clear, self.resume_r) + 0.5
            d_path, car_moving = self._sector_hit(cx, cy, cyaw, reach)
            # 「被挡」= 前方扇形内有车靠得太近，或者已经贴到车身（人顶着保险杠
            # 就是这种情况）。**不再区分车在不在动**：车停着也可能就是被我堵住
            # 的，靠「等 → 退 → 等」把路让开，比「忽略停着的车」更物理。
            #
            # [同学 C 本轮] 加上迟滞闩锁（见 LoopWalker.block_latch 的说明）。
            # 置位：扇形内 < evade_clear；解锁：必须躲到 resume_radius 以外并保持 0.6 s。
            if d_path < self.evade_clear:
                w.block_latch = True
                w.clear_t = 0.0
            elif d_path > self.resume_r:
                w.clear_t += self.dt
                if w.clear_t >= 0.6:
                    w.block_latch = False
            else:
                w.clear_t = 0.0      # 落在 [evade_clear, resume_radius] 之间：保持现状
            blocked = w.block_latch or (d_body < self.body_m + 0.10)

            # 预算：只有「确实往前走了一段」才复位，否则一次阻塞里能一直退下去
            ds = w.policy.sample(blocked, self.dt,
                                 budget_left=self.evade_max - w.back_used)
            # [同学 C 修复] 「那台车一直停着」也要把预算还回来。
            # 实测真机一轮：行人退到车侧 1.68 m 处把 15 m 预算磨光后站住，而自车
            # 的激光护栏也认为「车侧只剩 0.17 m」不敢过 —— 双方零错地僵持 190 s，
            # 路线超时。行人的预算本意是防「追着让」，但车根本没在动，
            # 让它再退一次才是打破僵局的低成本做法（只会让得多，不会让得少）。
            if blocked and not car_moving:
                w.static_t += self.dt
                if w.static_t > 12.0:
                    w.static_t = 0.0
                    if w.back_used > 0.0:
                        self.get_logger().info(
                            f'{w.name} 挡路的车一直没动，恢复后退预算再让一次')
                        w.back_used = 0.0
                        w.policy.mode = YieldPolicy.WALK   # 重新走一遍让行序列
            else:
                w.static_t = 0.0
            if ds > 1e-9:
                w.walk_clear += ds
                if w.walk_clear > 3.0:
                    w.back_used = 0.0
            elif ds < -1e-9:
                w.walk_clear = 0.0
            if ds < 0.0:
                # 后退方向也要看：不能倒进后面的车。会被撞就放弃后退，直接站住。
                nx, ny, _ = w.at(w.s + ds)
                if self._body_dist(nx, ny) < self.body_m:
                    w.policy.abort_back()
                    ds = 0.0
                else:
                    w.back_used += -ds
            x, y, yaw = w.advance(ds) if abs(ds) > 1e-9 else w.at(w.s)

            # [同学 C 修复] 让人**侧向挪开**，不能只沿自己的路后退。
            # 用户截图里的僵局：蓝色干扰车停着等，蓝色行人站着不动。原因是两边
            # 的「让行距离」对不上 ——
            #   · 车：`ped_stop = 7.0 m`（行人落在前方走廊里就等）；
            #   · 人：`evade_clear = 1.6 m`（车体离自己路径 1.6 m 内才让）。
            # 于是在 1.6~7.0 m 这个区间里人觉得自己没被挡、车觉得被堵，谁都不动。
            # 而且人的后退是**沿着自己的闭环**退的，方向常常和车的走廊平行，
            # 退 2 m 之后仍然在车的等待范围内，退多少次都没用。
            # 现在：只要被判为「被挡」，就朝**背离那台车**的一侧横移 side_step 米
            # （> 车的走廊半宽 2.4 m），从几何上离开车的行进带子；车开走后再回位。
            if blocked:
                car = self._nearest_vehicle_pose(x, y)
                if car is not None:
                    # [同学 C 本轮] 侧移量改为**直接对着车的走廊算**，不再固定 2.8 m。
                    # 旧写法：朝「背离车」的一侧移动 `side_step`，方向取自己路径的
                    # 左法向投影。问题是有时路径与车的轴线**平行** —— 这时 2.8 m
                    # 的侧移沿车的轴向走，一点用都没有，人退开 2.8 m 仍在车头正前方。
                    # 现在先把人换到**车体坐标**下量 `lat_c`，算出「还差多少才出去」
                    # （车走廊半宽 + 0.5 m），再投影到自己路径的左法向上落成 `side`。
                    c, s = math.cos(car[2]), math.sin(car[2])
                    dxc, dyc = x - car[0], y - car[1]
                    lat_c = -s * dxc + c * dyc
                    sgn = 1.0 if lat_c >= 0.0 else -1.0
                    need = self.cor_clear - abs(lat_c)
                    nx, ny = -math.sin(yaw), math.cos(yaw)   # 自己路径的左法向
                    dot = sgn * (nx * (-s) + ny * c)
                    if need > 1e-3 and abs(dot) > 0.25:
                        w.side_target = max(-3.6, min(3.6, need * dot))
                    else:
                        w.side_target = sgn * self.side_step
            else:
                w.side_target = 0.0
            step = self.side_speed * self.dt
            if w.side < w.side_target - 1e-6:
                w.side = min(w.side_target, w.side + step)
            elif w.side > w.side_target + 1e-6:
                w.side = max(w.side_target, w.side - step)
            if abs(w.side) > 1e-6:
                nx, ny = -math.sin(yaw), math.cos(yaw)
                sx, sy = x + nx * w.side, y + ny * w.side
                # 挪过去的那一侧也得是空着的，否则宁可不挪
                if self._body_dist(sx, sy) >= self.body_m:
                    x, y = sx, sy
                else:
                    w.side = 0.0
            self._log_yield(w, d_path, d_body, blocked, w.side)
            self.send_pose(w.name, x, y, yaw)
            poses.append((x, y, yaw))
        self.publish_states(poses)

    def _log_yield(self, w, d_path, d_body, blocked, side=0.0):
        """只在状态**变化**时打一条，避免每帧刷屏。"""
        mode = w.policy.mode
        if getattr(w, '_log_mode', None) == mode:
            return
        w._log_mode = mode
        if mode == YieldPolicy.WAIT:
            self.get_logger().info(
                f'{w.name} 被挡（前方扇形内 {d_path:.1f} m、离车体 {d_body:.1f} m），'
                f'原地等待 {w.policy.wait_before_back:.1f} s 看车是否先走')
        elif mode == YieldPolicy.BACK:
            self.get_logger().info(
                f'{w.name} 仍被挡，沿原路径主动后退 {w.policy.back_dist:.1f} m 让行'
                f'（同时向左/右让开 {side:+.1f} m）')
        elif mode == YieldPolicy.HOLD:
            self.get_logger().info(
                f'{w.name} 已退 {w.policy.back_dist:.1f} m，再等待 '
                f'{w.policy.hold_after_back:.1f} s 后重新尝试前进')
        elif mode == YieldPolicy.WALK and blocked is False:
            self.get_logger().info(f'{w.name} 重新起步前进')

    def publish_states(self, poses):
        """把行人的位姿广播出去（干扰车用它让行）。

        帧号必须写 `world`：这些值就是 gz 世界坐标（闭环航点直接 set_pose 过去的），
        而消费方之一 `path_agent` 也是拿 gz 里程计直接比。规划层要 map 帧，
        它在 `on_pedestrians` 里自己减 spawn 变换（见 planner_node）。
        """
        arr = PoseArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = 'world'
        for (x, y, yaw) in poses:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(self.z)
            p.orientation.z = math.sin(yaw * 0.5)
            p.orientation.w = math.cos(yaw * 0.5)
            arr.poses.append(p)
        self.pub_states.publish(arr)

    def send_pose(self, name, x, y, yaw):
        req = SetEntityPose.Request()
        req.entity.name = name
        req.entity.type = 2                       # MODEL
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = self.z
        p.orientation.z = math.sin(yaw * 0.5)
        p.orientation.w = math.cos(yaw * 0.5)
        req.pose = p
        self.cli.call_async(req)


def main(args=None):
    rclpy.init(args=args)
    node = PedestrianAgent()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
