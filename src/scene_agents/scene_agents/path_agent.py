"""场景干扰物的公共航点跟踪器（闭环 + 随机起点 + 遇车停车）。

为什么单独实现而不复用 parking_planner 的跟踪器：干扰车只走「道路中心线上的
折线」，不需要横向/航向反馈那一套精细控制；而且让场景层依赖规划层会形成反向
耦合。这里保持精简、只依赖 rclpy。

坐标：干扰车的 `/xxx/odom_gt` 发布的是**世界坐标绝对位姿**（OdometryPublisher
直接由仿真真值给出），因此航点也直接写世界坐标，无需再变换。

相对上一版的三处改动
--------------------
1. **闭环**：航点表按「首尾相接」书写，走到最后一个点后 idx 归零继续，
   不再「到终点停车再重来」。
2. **随机起点**：开局把车用 gz 的 `set_pose` 服务搬到闭环上的随机弧长处，
   并把跟踪索引同步过去，所以每次仿真的初始分布都不一样。
3. **遇车停车**：`stop_radius` 内有其它车辆就刹停（发布零速），车走开再继续。
4. **防死锁**：连续让行超过 `unblock_wait` 秒且挡路的车一直没动（位移
   < 0.3 m）时，确认身后安全后主动倒退 `unblock_back` 米把路让出来 ——
   否则会和自车互相让行到超时（真机 E2 实测：自车横在过道里等干扰车、
   干扰车等自车，双方僵持到整轮超时）。与行人 YieldPolicy 的
   「等待 → 退让 → 再等待」是同一套思路。
"""

import math
import random

import rclpy
from geometry_msgs.msg import Pose, PoseArray, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


# 车辆外廓（与 lot_layout.INCLUDE_FOOTPRINTS / 模型 sdf 一致）
CAR_HL, CAR_HW = 4.35 * 0.5, 1.85 * 0.5


def rects_overlap(ax, ay, ayaw, ahl, ahw, bx, by, byaw, bhl, bhw):
    """两个**有向矩形**是否重叠（分离轴定理）。

    [同学 C] 让行判据必须按车身矩形求交，不能再用「横向 |lat| < 车道半宽」这种
    中心点判据。真机截图里的那起碰撞就是这么来的：自车在过道里横着（航向差约
    90°），旧的判据里有一条「距离 > 5 m 且航向差 > 60° 就当成另一条车道，直接
    不看了」，于是蓝色干扰车一路开到 5 m 才开始理它 —— 而两台 4.35 m 长的车
    横着贴上去，中心距 3.1 m 就已经碰上了。
    SAT 只有 4 条轴，比采样点判据既准又快。
    """
    axes = []
    for yaw in (ayaw, byaw):
        c, s = math.cos(yaw), math.sin(yaw)
        axes.append((c, s))
        axes.append((-s, c))
    dx, dy = bx - ax, by - ay
    for (ux, uy) in axes:
        ca, sa = math.cos(ayaw), math.sin(ayaw)
        ra = ahl * abs(ca * ux + sa * uy) + ahw * abs(-sa * ux + ca * uy)
        cb, sb = math.cos(byaw), math.sin(byaw)
        rb = bhl * abs(cb * ux + sb * uy) + bhw * abs(-sb * ux + cb * uy)
        if abs(dx * ux + dy * uy) > ra + rb:
            return False
    return True



def fillet(points, radius=5.0, step=0.25, closed=True):
    """折线转角用二次 Bezier 圆滑，再按 step 等距重采样。

    closed=True 时把首点也当成一个转角处理（保证环闭合处也是圆滑的）。
    """
    pts = list(points)
    if closed and len(pts) >= 3:
        pts = [pts[-1]] + pts + [pts[0]]
    if len(pts) < 3:
        dense = list(pts)
    else:
        out = [pts[0]]
        for i in range(1, len(pts) - 1):
            p0, p1, p2 = pts[i - 1], pts[i], pts[i + 1]
            v1 = (p0[0] - p1[0], p0[1] - p1[1])
            v2 = (p2[0] - p1[0], p2[1] - p1[1])
            l1 = math.hypot(*v1)
            l2 = math.hypot(*v2)
            if l1 < 1e-6 or l2 < 1e-6:
                continue
            t = min(radius, 0.45 * l1, 0.45 * l2)
            a = (p1[0] + v1[0] / l1 * t, p1[1] + v1[1] / l1 * t)
            b = (p1[0] + v2[0] / l2 * t, p1[1] + v2[1] / l2 * t)
            out.append(a)
            for k in range(1, 8):
                u = k / 8.0
                out.append(((1 - u) ** 2 * a[0] + 2 * (1 - u) * u * p1[0] + u ** 2 * b[0],
                            (1 - u) ** 2 * a[1] + 2 * (1 - u) * u * p1[1] + u ** 2 * b[1]))
            out.append(b)
        out.append(pts[-1])
        dense = out

    resampled = []
    for i in range(len(dense) - 1):
        x0, y0 = dense[i]
        x1, y1 = dense[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(seg / step))
        for k in range(n):
            u = k / n
            resampled.append((x0 + (x1 - x0) * u, y0 + (y1 - y0) * u))
    resampled.append(dense[-1])
    return resampled


class PathAgent(Node):
    """沿世界坐标闭环折线行驶。"""

    def __init__(self, node_name, default_waypoints):
        super().__init__(node_name)
        self.declare_parameter('odom_topic', '/traffic/odom_gt')
        self.declare_parameter('cmd_topic', '/traffic/cmd_vel')
        self.declare_parameter('entity', 'traffic_car')
        self.declare_parameter('world', 'city_parking_lot')
        self.declare_parameter('waypoints', '')
        self.declare_parameter('speed', 2.0)
        self.declare_parameter('steer_gain', 1.4)
        # 前视距离：4.0 → 3.0。转角处前视太远会「切内角」压上路缘，
        # 实测两台车都在切角后开进路缘卡死。
        self.declare_parameter('lookahead', 3.0)
        self.declare_parameter('stop_radius', 9.0)
        # 让行判定：对本车**前方走廊**里与车身矩形相交的车让行（见 `_blocked`）。
        # 旧的「中心点 + 横向半宽 + 航向差豁免」三件套已经删掉 —— 它漏掉
        # 「横着停在过道里的车」，那正是真机截图里蓝色车撞上自车的原因。
        # corridor_half 是走廊半宽：自车半宽 0.925 + 约 1.5 m 余量。
        self.declare_parameter('corridor_half', 2.4)
        # 走廊的纵向起点（m，车体坐标）。取 0 = 从后轴中点往前算；负值会把
        # 「正后方的车」也框进来，实测那样会让本车为身后 2.4 m 的车白白停车
        # （旧的 `lon > -1` 版互锁就是这么来的）。车身自身覆盖的那一段由下面
        # 的「矩形重叠兜底」负责，不需要走廊来管。
        self.declare_parameter('corridor_back', 0.0)
        # 硬性最小间距：贴到这个距离以内无条件让行（兜底，与朝向无关）。
        self.declare_parameter('hard_radius', 4.0)
        # 随机起点的最小间隔：避免两台车出生就紧贴在一起。
        self.declare_parameter('spawn_gap', 12.0)
        # 延后多久再随机布点（秒）：让另一台车先放好，spawn_gap 才有参照物。
        self.declare_parameter('place_delay', 0.0)
        self.declare_parameter('vehicles', '/odom_gt')
        # 行人位姿话题 + 让行半径（离车体外廓的近似：这里用中心距，但 ped_stop
        # 给得足够大，等效于在行人前方几米就停）。
        self.declare_parameter('pedestrians', '/pedestrians')
        # 行人让行半径：**必须 ≥ 行人自身避障半径（3.5 m）的两倍**，这是用户明确
        # 提的要求（车让人的距离要比人让车的距离大一倍，否则车会「顶」到人身上
        # 才停）。5.5 → 7.0 满足 2×3.5。
        self.declare_parameter('ped_stop', 7.0)
        self.declare_parameter('ped_hard', 3.0)
        self.declare_parameter('seed', 0)
        self.declare_parameter('random_start', True)
        # ---------------- 防死锁（与行人 YieldPolicy 同一套思路）----------------
        # 让行多久、且挡路的车一直没动，才主动倒退打破僵持。12 s 与行人侧
        # 「车静止超 12 s 恢复让行预算」的阈值保持一致。
        # [本轮] 12 → 15 s：B3/SE1 真机上自车和干扰车反复互让，12 s 一轮的
        # 「等→退→再等」节奏追不上，多等 3 s 给自车留出更长的通过窗口。
        self.declare_parameter('unblock_wait', 15.0)
        # 一次倒退的距离 / 速度。12 m 保证**一次退出 10.5 m 的让行半径**，
        # 不再出现「退 4 m 仍落在耦合区里、退完又停」的反复（B3 真机日志）。
        self.declare_parameter('unblock_back', 12.0)
        self.declare_parameter('unblock_speed', 0.6)
        # 同一次僵持最多倒退几次：用完就站住死等，防止在走廊边缘来回蹭。
        # 只有真的前进超过 3 m（走出这段僵持区）才把次数还回来（对齐行人侧
        # `walk_clear > 3.0 → back_used = 0`）。
        self.declare_parameter('unblock_max', 3)
        # [本轮新增] 倒退结束后的**原地停留**时长（s）。退到让行半径之外后
        # 立刻前进会很快再次迎面撞上自车、重新触发让行；停够这段时间，
        # 自车有充足窗口通过这段车道（B3/SE1 的「多次让行」问题）。
        self.declare_parameter('unblock_hold', 15.0)

        raw = self.get_parameter('waypoints').value
        if raw:
            pts = []
            for item in raw.replace(';', ' ').split():
                x, y = item.split(',')
                pts.append((float(x), float(y)))
            self.points = pts if len(pts) >= 3 else list(default_waypoints)
        else:
            self.points = list(default_waypoints)

        self.speed = float(self.get_parameter('speed').value)
        self.gain = float(self.get_parameter('steer_gain').value)
        self.lookahead = float(self.get_parameter('lookahead').value)
        self.stop_r = float(self.get_parameter('stop_radius').value)
        self.cor_half = float(self.get_parameter('corridor_half').value)
        self.cor_back = float(self.get_parameter('corridor_back').value)
        self.hard_r = float(self.get_parameter('hard_radius').value)
        self.spawn_gap = float(self.get_parameter('spawn_gap').value)
        self.place_delay = float(self.get_parameter('place_delay').value)
        self.unblock_wait = float(self.get_parameter('unblock_wait').value)
        self.unblock_back = float(self.get_parameter('unblock_back').value)
        self.unblock_speed = float(self.get_parameter('unblock_speed').value)
        self.unblock_max = int(self.get_parameter('unblock_max').value)
        self.unblock_hold = float(self.get_parameter('unblock_hold').value)
        self._t_start = self.get_clock().now()
        self.path = fillet(self.points, radius=5.0, step=0.3, closed=True)

        self.idx = 0
        self.pose = None
        self.blocked = False
        self._hold = 0.0
        # 防死锁状态：_blocker 当前挡路车的位置（_blocked 顺带记录）；
        # _blk_t0/_blk_ref 本次阻塞开始的时刻与当时挡路车的位置；
        # _back_from 非空表示正在倒退让行（记录起点用于测距）；
        # _back_n 本次僵持已倒退次数；_yield_anchor 上次倒退结束时的位置，
        # 从那里前进超过 3 m 才把 _back_n 还回来。
        self._blocker = None
        self._blk_t0 = None
        self._blk_ref = None
        self._back_from = None
        self._back_n = 0
        self._yield_anchor = None
        # [本轮新增] 倒退结束后的原地停留截止时刻（None = 不在停留中）
        self._hold_until = None
        self._placed = not bool(self.get_parameter('random_start').value)
        seed = int(self.get_parameter('seed').value)
        self.rng = random.Random(seed if seed else None)

        self.pub = self.create_publisher(Twist, self.get_parameter('cmd_topic').value, 10)
        own = self.get_parameter('odom_topic').value
        self.create_subscription(Odometry, own, self.on_odom, 20)
        self.others = {}
        for topic in str(self.get_parameter('vehicles').value).split(','):
            topic = topic.strip()
            if topic and topic != own:
                self.create_subscription(
                    Odometry, topic, lambda m, t=topic: self.on_other(m, t), 20)
                self.others[topic] = None

        # 行人：由 pedestrian_agent 广播的 PoseArray。原来干扰车只认识「车」，
        # 会直接从行人身上压过去（实测把行人压到离车体 0.0 m）。
        self.peds = []
        self.ped_stop = float(self.get_parameter('ped_stop').value)
        self.ped_hard = float(self.get_parameter('ped_hard').value)
        ptopic = str(self.get_parameter('pedestrians').value).strip()
        if ptopic:
            self.create_subscription(PoseArray, ptopic, self.on_peds, 10)

        world = self.get_parameter('world').value
        self.entity = self.get_parameter('entity').value
        self.svc = f'/world/{world}/set_pose'
        self.cli = self.create_client(SetEntityPose, self.svc)

        self.create_timer(0.05, self.tick)
        self.get_logger().info(
            f'[{node_name}] 航点 {len(self.points)} 个，闭环重采样 {len(self.path)} 点，'
            f'速度 {self.speed:.1f} m/s，遇车停车半径 {self.stop_r:.1f} m')

    def on_odom(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        self.pose = (p.x, p.y, yaw)

    def on_other(self, msg, topic):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                         1 - 2 * (q.y * q.y + q.z * q.z))
        self.others[topic] = (p.x, p.y, yaw)

    def on_peds(self, msg):
        self.peds = [(p.position.x, p.position.y) for p in msg.poses]

    def _near_other(self, x, y):
        for pos in self.others.values():
            if pos is None:
                continue
            if math.hypot(pos[0] - x, pos[1] - y) < self.spawn_gap:
                return True
        return False

    # ------------------------------------------------------------ 随机起点
    def _place_random(self):
        """把车搬到闭环上的随机点，并把跟踪索引同步过去。

        额外约束：与其它车辆（含自车）至少相距 spawn_gap，避免两台干扰车
        「出生点紧密连接」后立刻在车道口互堵。
        """
        n = len(self.path)
        idx = self.rng.randrange(n)
        for _ in range(80):
            cand = self.rng.randrange(n)
            if not self._near_other(*self.path[cand]):
                idx = cand
                break
        x, y = self.path[idx]
        nxt = self.path[(idx + 1) % n]
        yaw = math.atan2(nxt[1] - y, nxt[0] - x)

        req = SetEntityPose.Request()
        req.entity.name = self.entity
        req.entity.type = 2                      # MODEL
        p = Pose()
        p.position.x = float(x)
        p.position.y = float(y)
        p.position.z = 0.05
        p.orientation.z = math.sin(yaw * 0.5)
        p.orientation.w = math.cos(yaw * 0.5)
        req.pose = p
        self.cli.call_async(req)

        self.idx = idx
        self.pose = (x, y, yaw)
        self._placed = True
        self.get_logger().info(
            f'[{self.get_name()}] 随机起点：{self.entity} 放到 '
            f'({x:.1f}, {y:.1f})，闭环进度 {idx}/{n}')

    # ---------------------------------------------------------------- 主循环
    def _nearest_ahead(self, x, y, window=80):
        best, bd = self.idx, float('inf')
        for i in range(self.idx, min(self.idx + window, len(self.path))):
            d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
            if d < bd:
                bd, best = d, i
        # 偏离路径的救援：本地窗口里找不到 3 m 以内的点，说明车已经开偏
        # （贴上路缘 / 被挤出去）。此时原来会继续朝着一个很远的旧前视点开，
        # 越开越偏直至卡死 —— 实测 A 卡在 (24.4,3.8)、B 卡在 (28.0,15.0)，
        # 都在路缘上，日志里没有任何「停车让行」，就是这个问题。
        # 这里允许**全局重定位**（可以往后找），把车拉回最近的路径点。
        if bd > 3.0:
            for i in range(len(self.path)):
                d = math.hypot(self.path[i][0] - x, self.path[i][1] - y)
                if d < bd:
                    bd, best = d, i
            return best, bd
        return best, bd

    def _blocked(self, x, y, th):
        """对**本车前方走廊里有车身**的车/人让行。

        [同学 C 修复] 判据从「中心点距离 + 横向半宽 + 航向差豁免」改成
        **车身矩形求交**：
          * 走廊 = 车体坐标下的矩形 `lon ∈ [corridor_back, stop_radius]`、
            `lat ∈ ±corridor_half`，也就是「本车往前开这一路会扫过的带子」；
          * 对方车按其真实朝向铺一个 4.35×1.85 的矩形，和走廊做 SAT 求交。

        旧判据有两个致命漏洞，真机截图里的碰撞两者都踩了：
          1. `dist > 5.0 且 |Δyaw| > 60° → continue`（当成另一条车道）。自车在
             过道里横着停车时航向差正好 ~90°，于是蓝色车跑到 5 m 才开始理它；
             而两台 4.35 m 的车横着贴上去，中心距 3.1 m 就已经碰到了。
          2. `硬性间距 hard_radius = 3.0 m` 是**中心距**，比两车半长之和
             （4.35 m）还小 —— 这个兜底永远不会先于物理接触触发。
        现在没有航向豁免、硬性间距按矩形算，横向/斜向/对向都一视同仁。
        """
        c, s = math.cos(th), math.sin(th)
        lc = 0.5 * (self.cor_back + self.stop_r)
        chl = 0.5 * (self.stop_r - self.cor_back)
        cx, cy = x + c * lc, y + s * lc
        self._blocker = None      # 触发本次让行的车；行人触发的保持 None（见 _try_unblock）
        for pos in self.others.values():
            if pos is None:
                continue
            ox, oy, oyaw = pos
            if math.hypot(ox - x, oy - y) > self.stop_r + 4.0:
                continue
            if rects_overlap(cx, cy, th, chl, self.cor_half,
                             ox, oy, oyaw, CAR_HL, CAR_HW):
                self._blocker = (ox, oy)
                return True
            # 兜底：已经和自己的车身重叠了（比如被别人从侧面撞上），立刻停车。
            # 这条只会在「已经接触」时触发，所以不会像旧的 `lon > -1` 版本那样
            # 把「后方来车」误判成障碍造成互锁。
            if (math.hypot(ox - x, oy - y) < self.hard_r
                    and rects_overlap(x, y, th, CAR_HL, CAR_HW,
                                      ox, oy, oyaw, CAR_HL, CAR_HW)):
                self._blocker = (ox, oy)
                return True
        # 行人：很近（任意方向）或者在本车走廊前方就让行。行人只有 1.1 m/s，
        # 给足刹车距离才能避免「车从人身上压过去」。行人是点目标，按距离判即可。
        for (px, py) in self.peds:
            dx, dy = px - x, py - y
            dist = math.hypot(dx, dy)
            if dist < self.ped_hard:
                return True
            lon = c * dx + s * dy
            lat = -s * dx + c * dy
            if -1.5 < lon < self.ped_stop and abs(lat) < self.cor_half:
                return True
        return False

    def _rear_clear(self, x, y, th):
        """倒退安全检查：正后方走廊里没有车、身边没有行人。

        检查深度 = 倒退距离 + 1.5 m 余量；走廊半宽沿用 corridor_half。
        行人按任意方向 ped_hard（3 m）判 —— 行人是点目标且可能绕到车后，
        宁可不退也不能压到人。
        """
        c, s = math.cos(th), math.sin(th)
        depth = self.unblock_back + 1.5
        cx, cy = x - c * depth * 0.5, y - s * depth * 0.5
        for pos in self.others.values():
            if pos is None:
                continue
            ox, oy, oyaw = pos
            if rects_overlap(cx, cy, th, 0.5 * depth, self.cor_half,
                             ox, oy, oyaw, CAR_HL, CAR_HW):
                return False
        for (px, py) in self.peds:
            if math.hypot(px - x, py - y) < self.ped_hard:
                return False
        return True

    def _try_unblock(self, x, y, th):
        """僵持判定：让行超时且对方一直没动 → 主动倒退，把路让出来。

        三个前提缺一不可：
          1. 让行持续超过 `unblock_wait`（12 s）；
          2. 挡路的是**车**且从阻塞开始至今位移 < 0.3 m（对方在挪车/泊车
             时继续等就行，倒车反而会打断对方）；
          3. 身后走廊是空的（`_rear_clear`），否则退了就撞上后面的车。

        行人挡路不触发倒退：行人有自己的 YieldPolicy（等待→退让→再等待），
        车这边只管等。倒退次数有 `unblock_max` 上限，用完就站住死等。
        """
        if self._blk_t0 is None or self._back_n >= self.unblock_max:
            # [本轮修复] **退让次数用尽后仍要能再退**。
            # 真机 B3 a3：西街只有 8 m 宽，自车和干扰车正面相遇。干扰车退 12 m
            # + 停 15 s，自车前进 4 m 又被自己的激光护栏挡住（护栏 4.31 m），
            # 干扰车于是又构成阻塞 —— 但 `_back_n` 只有在「前进超过 3 m」时才
            # 复位，而它永远前进不了 3 m，于是 3 次用尽后**永久对顶 716 s**
            # （日志 `激光护栏 ... 已连续 716 s`，同期干扰车再无任何倒退）。
            # 处理：僵持累计满 3×unblock_wait（45 s）就当新一轮僵持，重置计数。
            # [B3 修复调整] 2× → 3×：复位太快（30 s）会让干扰车在自车还没抓住
            # 通过窗口时就再次前进挡路，形成「退-进-挡」的紧循环；45 s 一轮
            # 的节奏与自车入位段的冻结等待（planner 的 AGENT_FREEZE_MAX）匹配。
            # 复位本身必须保留：删掉它会回到「预算用尽永久站死」的对顶死锁。
            if self._back_n >= self.unblock_max and self._blk_t0 is not None:
                since = (self.get_clock().now() - self._blk_t0).nanoseconds * 1e-9
                if since > 3.0 * self.unblock_wait:
                    self.get_logger().info(
                        f'[{self.get_name()}] 僵持累计 {since:.0f} s，'
                        f'退让次数复位（原 {self._back_n}/{self.unblock_max}）')
                    self._back_n = 0
                else:
                    return
            else:
                return
        held = (self.get_clock().now() - self._blk_t0).nanoseconds * 1e-9
        if held < self.unblock_wait:
            return
        if self._blocker is None or self._blk_ref is None:
            return
        moved = math.hypot(self._blocker[0] - self._blk_ref[0],
                           self._blocker[1] - self._blk_ref[1])
        if moved > 0.3:
            # [本轮修复] 对方还在动 → **重新计时**。
            # 旧实现「位移 > 0.3 m 就永远不倒退」：_blk_ref 是阻塞**开始**时的
            # 位置，自车蠕行 2 m（哪怕之后静止 240 s），累计位移永远 > 0.3 m，
            # 干扰车于是永远等下去 —— 真机 B3 a2 的 240 s 死锁就是这么来的
            # （自车护栏 4 m 外顶住，干扰车「前方 10.5 m 内有车」等了一路）。
            # 改成「对方最近动过就重新计时」：静止持续满 unblock_wait 才倒退。
            self._blk_t0 = self.get_clock().now()
            self._blk_ref = self._blocker
            return
        if not self._rear_clear(x, y, th):
            return
        self._back_n += 1
        self._back_from = (x, y)
        self.get_logger().info(
            f'[{self.get_name()}] 让行 {held:.0f} s 且对方未动（位移 {moved:.2f} m），'
            f'第 {self._back_n}/{self.unblock_max} 次倒退 {self.unblock_back:.1f} m 打破僵持')

    def tick(self):
        if self.pose is None:
            return
        if not self._placed:
            if not self.cli.service_is_ready():
                return
            if self.place_delay > 0.0:
                age = (self.get_clock().now() - self._t_start).nanoseconds * 1e-9
                if age < self.place_delay:
                    return
            self._place_random()
            return

        x, y, th = self.pose
        self.idx, _ = self._nearest_ahead(x, y)

        # [本轮新增] 倒退后的原地停留：停够 unblock_hold 秒再恢复巡航，
        # 给自车留出完整通过窗口（否则退完马上前进，会再次迎面触发让行）。
        # [B3 修复] 停留期间每帧复查走廊：挡路源已经离开（自车泊完车走了）
        # 就提前恢复巡航，不死等满 unblock_hold —— 白等的这段时间里自车
        # 若又需要这条车道，反而多等一轮。
        if self._hold_until is not None:
            if self.get_clock().now() < self._hold_until:
                if self._blocked(x, y, th):
                    self.pub.publish(Twist())
                    return
                self.get_logger().info(
                    f'[{self.get_name()}] 停留期间走廊已清，提前恢复巡航')
            else:
                self.get_logger().info(
                    f'[{self.get_name()}] 倒退后停留 {self.unblock_hold:.0f} s 结束，恢复巡航')
            self._hold_until = None

        # 倒退让行执行中：退够 unblock_back 米为止。期间只看身后（_rear_clear），
        # 不看前方 —— 前方本来就堵着才退的；身后被占就放弃本次，站住接着等
        # （下个周期满 unblock_wait 且条件仍满足时还能再试）。
        if self._back_from is not None:
            if not self._rear_clear(x, y, th):
                self._back_from = None
                self.pub.publish(Twist())
                return
            done = math.hypot(x - self._back_from[0], y - self._back_from[1])
            if done >= self.unblock_back:
                self._back_from = None
                self._yield_anchor = (x, y)
                # 重新计时：再满一轮 unblock_wait 才允许下一次倒退
                self._blk_t0 = self.get_clock().now()
                self._blk_ref = self._blocker
                # [本轮新增] 退到位后原地停留，把车道让给自车通过
                self._hold_until = (self.get_clock().now()
                                    + rclpy.time.Duration(
                                        seconds=self.unblock_hold))
                self.get_logger().info(
                    f'[{self.get_name()}] 倒退 {done:.1f} m 完成，原地停留 '
                    f'{self.unblock_hold:.0f} s 让自车通过')
                self.pub.publish(Twist())
                return
            tw = Twist()
            tw.linear.x = float(-self.unblock_speed)
            self.pub.publish(tw)
            return

        # 遇车停车：半径内出现车辆就刹停，车走开继续
        blk = self._blocked(x, y, th)
        if blk != self.blocked:
            self.blocked = blk
            if blk:
                self._blk_t0 = self.get_clock().now()
                self._blk_ref = self._blocker
                self.get_logger().info(
                    f'[{self.get_name()}] 前方 {self.stop_r:.1f} m 内有车，停车让行',
                    throttle_duration_sec=2.0)
        if self.blocked:
            self._try_unblock(x, y, th)
            self.pub.publish(Twist())
            return

        # 从上一次倒退让行的位置真的前进了一段（> 3 m）：僵持已解除，
        # 把倒退次数还回来，下次僵持可以重新用满 unblock_max 次。
        if (self._yield_anchor is not None
                and math.hypot(x - self._yield_anchor[0],
                               y - self._yield_anchor[1]) > 3.0):
            self._yield_anchor = None
            self._back_n = 0

        # 闭环：走到末尾就回到第 0 点（航点表首尾相接，所以是连续的）
        if self.idx >= len(self.path) - 2:
            self.idx = 0

        # 前视点
        ld = self.lookahead
        li = self.idx
        for i in range(self.idx, len(self.path)):
            if math.hypot(self.path[i][0] - x, self.path[i][1] - y) >= ld:
                li = i
                break
        else:
            li = len(self.path) - 1
        cx, cy = self.path[li]
        d = math.hypot(cx - x, cy - y)
        alpha = wrap(math.atan2(cy - y, cx - x) - th)
        kappa = 2.0 * math.sin(alpha) / max(d, 0.5)
        kappa = max(-0.396, min(0.396, kappa))       # 与实测最小转弯半径匹配

        tw = Twist()
        tw.linear.x = float(self.speed)
        tw.angular.z = float(self.speed * kappa * self.gain)
        self.pub.publish(tw)
