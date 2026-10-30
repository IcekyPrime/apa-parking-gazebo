"""Hybrid A*：在 (x, y, θ) 上做满足阿克曼运动学的启发式搜索。

要点
----
* 运动基元：固定曲率（对应若干前轮转角）× 前进/后退，按弧长积分；
* 碰撞：车体用「沿纵轴的 7 个圆」近似（半径 0.95 m，正好覆盖 1.85 m 车宽），
  与世界模型的膨胀栅格逐点比对；
* 启发：2D 测地距离场（考虑障碍、忽略运动学），可采纳；
* 解析扩展：接近目标时用 Reeds-Shepp 曲线直接连到终点，逼近最优并保证换挡可达；
* 代价：行驶距离 + 倒车惩罚 + 换挡惩罚 + 转角变化惩罚。

[同学 C 本轮 §6.29] **参考点 = 后轴**。`motion()` 一直是后轴积分，但碰撞轮廓
原来以位姿原点为中心对称 —— 等于把「中心位姿」喂给后轴积分器、再当中心查
碰撞，两套坐标系混用。后果在真机 E2 上显形：中心点弧线运动时速度恒偏航向
β=atan(κ·L/2)≈12°（κ=0.165），控制器把航向压到切线、速度仍斜着走（e 以
0.17/m 单调发散）；加 β 补偿后 e 收敛，但 C|C 曲率变号点处「速度方向」需求
瞬跳 2β≈25°，而执行器曲率余量只有 0.037 rad/m —— R 段全程漂移追不回。
后轴参考的速度**恒沿车体纵轴**：零 β、变号点零速度方向跳变，cusp 处也只剩
「曲率指令翻转」这一个二阶问题（换挡停车 + flip_preload 可兜）。
调用方约定：`plan()/free()` 收**后轴位姿**（planner_node 用 `_rear()` 从中心
位姿换算，控制器侧 `PathFollower(ref_ahead=0)` 对应换算反馈位姿）。
"""

import heapq
import math

from scipy import ndimage

from .geometry import wrap
from .reeds_shepp import reeds_shepp

# 车体包围圆：沿纵向 7 个，半径 0.95 m，能覆盖 4.35 m x 1.85 m 的矩形。
# [§6.29] 圆心相对**后轴**分布：原点 = 后轴，车尾在 -0.825、车头在 +3.525
# （前后悬对称，AXLE_OFFSET = 轴距/2 = 1.35 即「中心 → 后轴」的纵移量；
# 车长固定 4.35，若换 wheelbase 需同步这里与 BODY_REAR/BODY_FRONT）。
AXLE_OFFSET = 1.35
CIRCLE_RADIUS = 0.95
CIRCLE_OFFSETS = [-1.225 + 2.45 * i / 6.0 + AXLE_OFFSET for i in range(7)]

# 车体轮廓采样点（车体系，原点=后轴）：4 角 + 长边每 ~0.4 m 一点。配合原始
# 栅格的 EDT 用 —— 「到最近占据格的距离」就是该点到障碍的真实距离，对整个轮廓
# 取最小即真实矩形间隙。7 圆模型在长边处偏乐观（实测 W7 有 circle 0.402 /
# body 0.054 的解），按圆余量选解会选错，所以选解用这个。
BODY_REAR, BODY_FRONT, BODY_HW = -0.825, 3.525, 0.925


def _body_outline(step=0.40):
    pts = [(BODY_REAR, -BODY_HW), (BODY_REAR, BODY_HW),
           (BODY_FRONT, -BODY_HW), (BODY_FRONT, BODY_HW)]
    n = max(1, int(round((BODY_FRONT - BODY_REAR) / step)))
    for i in range(1, n):
        x = BODY_REAR + (BODY_FRONT - BODY_REAR) * i / n
        pts += [(x, -BODY_HW), (x, BODY_HW)]
    m = max(1, int(round(2 * BODY_HW / step)))
    for j in range(1, m):
        y = -BODY_HW + 2 * BODY_HW * j / m
        pts += [(BODY_REAR, y), (BODY_FRONT, y)]
    return pts


BODY_OUTLINE = _body_outline()


def motion(pose, curvature, ds):
    """按曲率 curvature 前进（带符号）弧长 ds，返回新位姿。"""
    x, y, th = pose
    if abs(curvature) < 1e-9:
        return (x + ds * math.cos(th), y + ds * math.sin(th), th)
    th2 = th + ds * curvature
    return (x + (math.sin(th2) - math.sin(th)) / curvature,
            y - (math.cos(th2) - math.cos(th)) / curvature,
            wrap(th2))


class PlanResult:
    def __init__(self, success, path=None, cost=0.0, expansions=0, reason="",
                 used_rs=False, length=0.0, reverse_length=0.0, n_switch=0):
        self.success = success
        self.path = path or []          # [(x, y, yaw, direction, curvature)]
        self.cost = cost
        self.expansions = expansions
        self.reason = reason
        self.used_rs = used_rs
        self.length = length
        self.reverse_length = reverse_length
        self.n_switch = n_switch

    def __repr__(self):
        if not self.success:
            return f"PlanResult(FAIL, {self.reason}, expansions={self.expansions})"
        return (f"PlanResult(OK, len={self.length:.2f} m, rev={self.reverse_length:.2f} m, "
                f"switch={self.n_switch}, cost={self.cost:.2f}, "
                f"exp={self.expansions}, rs={self.used_rs})")


class HybridAStar:
    def __init__(self, grid, wheelbase=2.70, max_steer=0.61,
                 xy_res=0.25, th_res=math.radians(3.0), step=0.55,
                 steer_ratios=(-1.0, -0.5, 0.0, 0.5, 1.0),
                 reverse_penalty=1.0, switch_penalty=2.0, steer_penalty=0.3,
                 xy_tol=0.30, th_tol=math.radians(6.0), max_iter=20000,
                 analytic_every=20, analytic_range=None, clearance=0.0,
                 max_cusps=2, start_free_radius=0.60, clear_target=0.0):
        self.grid = grid
        self.wheelbase = float(wheelbase)
        self.max_steer = float(max_steer)
        self.xy_res = float(xy_res)
        self.th_res = float(th_res)
        self.step = float(step)
        self.steers = [r * self.max_steer for r in steer_ratios]
        self.reverse_penalty = float(reverse_penalty)
        self.switch_penalty = float(switch_penalty)
        self.steer_penalty = float(steer_penalty)
        self.xy_tol = float(xy_tol)
        self.th_tol = float(th_tol)
        self.max_iter = int(max_iter)
        self.analytic_every = int(analytic_every)
        self.analytic_range = float(analytic_range or 3.0 * self.wheelbase)
        self.clearance = float(clearance)
        self.max_cusps = int(max_cusps)
        # 起点放宽半径：车已经站在这里了，重叠是既成事实，不是要避开的量。
        # 只要求「离开起点 start_free_radius 米之后」的路径点严格无碰撞。
        self.start_free_radius = float(start_free_radius)
        # 解析解的真实车体余量目标（m）：> 0 时 start→goal 直连不再取「最短」，
        # 而是在全部 RS 模式族里按余量挑（见 _analytic_select）。0 = 旧行为。
        self.clear_target = float(clear_target)
        self._start = None
        self.last_start_relaxed = False

        # 碰撞栅格：把障碍物按车宽的一半膨胀，之后只需检查圆心的格子
        self._radius = CIRCLE_RADIUS + self.clearance
        self._fp_grid = grid.inflated(self._radius)
        self._field = None
        self._goal = None

    # ------------------------------------------------------------------ 基础查询
    def curvature_of(self, steer):
        return math.tan(steer) / self.wheelbase

    def free(self, x, y, yaw):
        c, s = math.cos(yaw), math.sin(yaw)
        g = self._fp_grid
        for off in CIRCLE_OFFSETS:
            if g.occupied(x + c * off, y + s * off):
                return False
        return True

    def free_step(self, x, y, yaw):
        """扩展 / 解析段用的碰撞判据：起点放宽半径内一律放行，之外严格判据。

        背景：车位净宽 2.5 m、车宽 1.85 m，两侧只剩 0.325 m；栅格按半个车宽
        （0.95 m）膨胀后，自车只要在车位口有十几厘米横向偏差，`free(start)`
        就恒为 False。硬判据会让「车已经在窄位里」这个局面永远规划不出结果，
        而真实车辆此刻本来就压着膨胀区。所以起点邻域放行、离开后收紧 —— 这是
        窄空间规划器的通用做法。
        """
        s = self._start
        if s is not None and math.hypot(x - s[0], y - s[1]) <= self.start_free_radius:
            return True
        return self.free(x, y, yaw)

    def min_clearance(self, path):
        """路径上车体（7 圆模型）到最近障碍的最小间隙（m），可为负。

        规划器内部的碰撞判据是二值的（占据/不占据），一次规划「成功」并不代表
        离障碍还有多少余量。实测几个垂直车位的路径最小间隙：W5 +0.575 m、
        W9 只有 **+0.083 m** —— 后者只要跟踪误差稍大就会蹭上邻车。所以
        `planner_node` 规划完要用这个值复核一遍。

        注意用**原始栅格**的距离变换，再减掉圆半径：`_fp_grid` 已经膨胀过，
        在它上面算出来恒为 0。
        """
        if not path:
            return float("inf")
        free = self.grid.data == 0
        dist = ndimage.distance_transform_edt(free) * self.grid.res
        worst = float("inf")
        for p in path:
            x, y, th = p[0], p[1], p[2]
            c, s = math.cos(th), math.sin(th)
            for off in CIRCLE_OFFSETS:
                ix, iy = self.grid.to_index(x + c * off, y + s * off)
                if not self.grid.inside(ix, iy):
                    continue
                worst = min(worst, float(dist[iy, ix]) - CIRCLE_RADIUS)
        return worst

    def body_clearance(self, path, dist=None):
        """路径上**真实车体矩形**到障碍的最小间隙（m），EDT + 轮廓采样。

        与 min_clearance（7 圆模型）的差别：圆在车角处保守 ~0.1 m、长边处
        又偏乐观 —— 按圆余量挑解会把 W7 的 circle 0.402 / body 0.054 解
        当成安全解。选解（_analytic_select）用这个量。
        """
        if not path:
            return float("inf")
        if dist is None:
            free = self.grid.data == 0
            dist = ndimage.distance_transform_edt(free) * self.grid.res
        worst = float("inf")
        for p in path:
            x, y, th = p[0], p[1], p[2]
            c, s = math.cos(th), math.sin(th)
            for (lx, ly) in BODY_OUTLINE:
                wx, wy = x + c * lx - s * ly, y + s * lx + c * ly
                ix, iy = self.grid.to_index(wx, wy)
                if not self.grid.inside(ix, iy):
                    continue
                worst = min(worst, float(dist[iy, ix]))
        return worst

    def _key(self, x, y, th):
        ix = int(round((x - self.grid.xmin) / self.xy_res))
        iy = int(round((y - self.grid.ymin) / self.xy_res))
        ith = int(round(wrap(th) / self.th_res)) % int(round(2 * math.pi / self.th_res))
        return (ix, iy, ith)

    def _heuristic(self, x, y):
        h = self.grid.sample_field(self._field, x, y)
        if not math.isfinite(h):
            return float("inf")
        return h

    # ---------------------------------------------------------------- 主搜索循环
    def plan(self, start, goal):
        self._goal = goal
        self._field = self.grid.distance_field((goal[0], goal[1]))
        self._start = start
        self.last_start_relaxed = False

        # 起点判据：**放宽**。自车此刻就在 start 上，与障碍重叠是既成事实；
        # 硬拒绝会让窄位里的车永远规划不出结果（历史日志里表现为
        # 「start 位姿被占据」→ 退回以预备位姿为起点 → 横向偏差 2.5~4.3 m）。
        if not self.free(*start):
            self.last_start_relaxed = True
        if not self.free(*goal):
            return PlanResult(False, reason="goal 位姿被占据（车体与障碍重叠）")

        gx, gy, gth = goal
        h0 = self._heuristic(start[0], start[1])
        if not math.isfinite(h0):
            if not self.last_start_relaxed:
                return PlanResult(False, reason="goal 与 start 在 2D 栅格上不连通")
            # 起点压在**真实**占据格上（不只是膨胀区），距离场在起点为 inf。
            # 放宽语义下仍允许规划：用直线距离给一个（可采纳性变弱的）启发，
            # 后面每个扩展点都在放宽半径之外，仍是严格判据。
            h0 = math.hypot(gx - start[0], gy - start[1])

        # 先试一次解析扩展：起点直连终点。clear_target>0 时按**真实车体余量**
        # 在全部 RS 模式族里挑解（见 _analytic_select），否则维持最短解。
        rs = self._analytic_select(start, goal) if self.clear_target > 0.0 \
            else self._analytic(start, goal)
        if rs is not None:
            path, length, revs, sw = rs
            return PlanResult(True, path, cost=length, expansions=0,
                              reason="analytic", used_rs=True, length=length,
                              reverse_length=revs, n_switch=sw)

        open_heap = []
        counter = 0
        heapq.heappush(open_heap, (h0, counter, start, 0.0, 0, 0))
        best_g = {self._key(*start): 0.0}
        parents = {}
        poses = {}
        dirs = {}
        closed = set()      # 已展开的栅格状态；没有它同一状态会被反复入堆，
                            # 平行车位揉库这类难例能把堆撑到 20 万条（几百 MB），
                            # 在 4 GB 的 WSL 里会被 OOM 杀掉。

        sk = self._key(*start)
        poses[sk] = start
        dirs[sk] = 1
        parents[sk] = None

        expansions = 0
        last_key = sk
        while open_heap and expansions < self.max_iter:
            f, _, pose, g, d, prev_key = heapq.heappop(open_heap)
            key = self._key(*pose)
            if g > best_g.get(key, float("inf")) + 1e-9:
                continue
            if key in closed:
                continue
            closed.add(key)
            if key != sk and key not in poses:
                poses[key] = pose
                dirs[key] = d
                parents[key] = prev_key
            expansions += 1
            last_key = key

            # 终止判断
            if (math.hypot(pose[0] - gx, pose[1] - gy) <= self.xy_tol
                    and abs(wrap(pose[2] - gth)) <= self.th_tol):
                path = self._reconstruct(key, poses, dirs, parents)
                length = sum(abs(math.hypot(b[0] - a[0], b[1] - a[1]))
                             for a, b in zip(path, path[1:]))
                revs = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                           for a, b in zip(path, path[1:]) if b[3] < 0)
                sw = sum(1 for a, b in zip(path, path[1:]) if a[3] != b[3])
                return PlanResult(True, path, cost=g, expansions=expansions,
                                  reason="goal", length=length,
                                  reverse_length=revs, n_switch=sw)

            # 解析扩展（接近目标时）
            if expansions % self.analytic_every == 0 and f < self._heuristic(
                    pose[0], pose[1]) + 2.5 * self.analytic_range:
                rs = self._analytic(pose, goal)
                if rs is not None:
                    rs_path, length, revs, sw = rs
                    path = self._reconstruct(key, poses, dirs, parents)
                    path += rs_path
                    total = sum(abs(math.hypot(b[0] - a[0], b[1] - a[1]))
                                for a, b in zip(path, path[1:]))
                    rev_all = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                                  for a, b in zip(path, path[1:]) if b[3] < 0)
                    sw_all = sum(1 for a, b in zip(path, path[1:]) if a[3] != b[3])
                    return PlanResult(True, path, cost=g + length,
                                      expansions=expansions, reason="analytic",
                                      used_rs=True, length=total,
                                      reverse_length=rev_all, n_switch=sw_all)

            for steer in self.steers:
                curv = self.curvature_of(steer)
                for d_new in (1, -1):
                    nxt = motion(pose, curv, d_new * self.step)
                    if not self.free_step(*nxt):
                        continue
                    nkey = self._key(*nxt)
                    step_cost = self.step * (1.0 + self.reverse_penalty if d_new < 0 else 1.0)
                    if d_new != d:
                        step_cost += self.switch_penalty
                    step_cost += self.steer_penalty * abs(steer) / self.max_steer
                    ng = g + step_cost
                    if ng >= best_g.get(nkey, float("inf")) - 1e-9:
                        continue
                    h = self._heuristic(nxt[0], nxt[1])
                    if not math.isfinite(h):
                        if not self.last_start_relaxed:
                            continue
                        h = math.hypot(gx - nxt[0], gy - nxt[1])
                    best_g[nkey] = ng
                    counter += 1
                    heapq.heappush(open_heap, (ng + h, counter, nxt, ng, d_new, key))
                    if nkey not in poses:
                        poses[nkey] = nxt
                        dirs[nkey] = d_new
                        parents[nkey] = key

        return PlanResult(False, reason=f"搜索耗尽（expansions={expansions}）",
                          expansions=expansions)

    # -------------------------------------------------------------------- 辅助
    def _analytic(self, pose, goal):
        """用 Reeds-Shepp 曲线连接 pose → goal，并做碰撞检查。"""
        radius = self.wheelbase / math.tan(self.max_steer)
        rs = reeds_shepp(pose, goal, radius, max_cusps=self.max_cusps, quick=True)
        if rs is None:
            return None
        samples = rs.sample(0.20)
        if not samples:
            return None
        # 曲线起始一侧允许短距离重叠（车已经站在那个位姿上了），离开后严格检查。
        # 这里用**距曲线起点**的距离而不是样点序号：换挡点在曲线内部，
        # 用序号会把固定 0.4 m 的豁免挪到错误的位置。
        r = self.start_free_radius
        for (x, y, th, _d, _c) in samples:
            if math.hypot(x - pose[0], y - pose[1]) <= r:
                continue
            if not self.free(x, y, th):
                return None
        path = [(x, y, th, d, c) for (x, y, th, d, c) in samples]
        revs = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(path, path[1:]) if b[3] < 0)
        sw = sum(1 for a, b in zip(path, path[1:]) if a[3] != b[3])
        return path, rs.length, revs, sw

    def _analytic_select(self, pose, goal):
        """start→goal 直连的解析解，按**真实车体余量**优先选择。

        背景：最短解不等于最安全解。实测 W5/W7/E3/E8 的最短解析解车体间隙
        只有 +0.068 m（执行后 +0.058 m，离 0.2 m 的真机判据差三倍），而同
        起终点的其它模式族（长 1.6~6 m、多 0~1 次换挡）能给到 +0.319 m。
        规则：
          * 余量 ≥ clear_target 的解里取（长度 + 换挡惩罚）最小者；
          * 都不达标时，仅当更安全的解比最短解余量高 ≥0.15 m 才替换
            （与 planner_node 的 SAFE_CLEAR_MARGIN 同一判据 —— 不为
            0.1 m 余量多揉两把）。
        全模式族 + 多初值的求解比 quick 模式慢（~1 s 量级），只在 plan()
        的首次直连用，搜索内部的逐节点解析扩展仍走 _analytic。
        """
        radius = self.wheelbase / math.tan(self.max_steer)
        cands = reeds_shepp(pose, goal, radius, max_cusps=self.max_cusps,
                            quick=False, all_candidates=True)
        if not cands:
            return None
        free = self.grid.data == 0
        dist = ndimage.distance_transform_edt(free) * self.grid.res
        r = self.start_free_radius
        best_safe = None            # 余量达标集合里的 (score, path, ...)
        best_short = None           # 全局最短解 (score, clr, path, ...)
        best_clear = None           # 最大余量解 (clr, score, path, ...)
        for rs in cands:
            samples = rs.sample(0.20)
            if not samples:
                continue
            ok = True
            for (x, y, th, _d, _c) in samples:
                if math.hypot(x - pose[0], y - pose[1]) <= r:
                    continue
                if not self.free(x, y, th):
                    ok = False
                    break
            if not ok:
                continue
            clr = self.body_clearance(samples, dist)
            score = rs.length + 2.0 * rs.n_cusps
            path = [(x, y, th, d, c) for (x, y, th, d, c) in samples]
            revs = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                       for a, b in zip(path, path[1:]) if b[3] < 0)
            sw = sum(1 for a, b in zip(path, path[1:]) if a[3] != b[3])
            item = (score, path, rs.length, revs, sw)
            if best_safe is None or score < best_safe[0]:
                if clr >= self.clear_target:
                    best_safe = item
            if best_short is None or score < best_short[0]:
                best_short = (score, clr, item)
            if best_clear is None or clr > best_clear[0] + 1e-9 or (
                    abs(clr - best_clear[0]) <= 1e-9 and score < best_clear[1]):
                best_clear = (clr, score, item)
        if best_safe is not None:
            return best_safe[1], best_safe[2], best_safe[3], best_safe[4]
        if best_short is not None:
            _s, clr_short, item = best_short
            if best_clear is not None and best_clear[0] >= clr_short + 0.15:
                item = best_clear[2]
            return item[1], item[2], item[3], item[4]
        return None

    def _reconstruct(self, key, poses, dirs, parents):
        out = []
        k = key
        while k is not None:
            x, y, th = poses[k]
            out.append((x, y, th, dirs.get(k, 1), 0.0))
            k = parents.get(k)
        out.reverse()
        return out
