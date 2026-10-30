"""Reeds-Shepp 曲线：允许前进/后退换挡的最短泊车机动。

实现方式
--------
不查 48 组闭式解表，而是「模式枚举 + 数值求解 + 积分校验」：

1. 枚举经典 RS 词法族（CSC / CCC / CCCC / CCSC / CCSCC，C 为定曲率圆弧、
   S 为直线，正负号表示前进或后退）；
2. 对每种模式，以各段长度为未知量，用 `scipy.optimize.least_squares` 求解
   「从起点出发积分到终点位姿」的边界值问题，多个初值并行尝试；
3. 用前向积分复核，残差超过 1e-6 的候选直接丢弃，保证输出的路径一定可达；
4. 在可行解中取总长最短者（含换挡惩罚）。

这样得到的曲线与闭式 RS 属于同一族（曲率半径固定为 R，允许倒车与换挡），
但实现可验证、模式可扩展，且换挡次数可直接统计。

单位说明：内部用「归一化坐标」（R=1），对外接口接收真实米制位姿与最小转弯半径。
"""

import math

import numpy as np
from scipy.optimize import least_squares

from .geometry import wrap

# 模式表：每项是一串 (kind, steer)，kind='C' 圆弧（steer=±1 表示左右）或 'S' 直线
PATTERNS = [
    # CSC：S 形，倒车入库最常用
    (("C", 1), ("S", 0), ("C", 1)),
    (("C", 1), ("S", 0), ("C", -1)),
    (("C", -1), ("S", 0), ("C", 1)),
    (("C", -1), ("S", 0), ("C", -1)),
    # CCC：连续同/异向圆弧
    (("C", 1), ("C", -1), ("C", 1)),
    (("C", -1), ("C", 1), ("C", -1)),
    # CCCC：两段 S 形串联
    (("C", 1), ("C", -1), ("C", 1), ("C", -1)),
    (("C", -1), ("C", 1), ("C", -1), ("C", 1)),
    # CCSC：圆弧-圆弧-直线-圆弧
    (("C", 1), ("C", -1), ("S", 0), ("C", 1)),
    (("C", 1), ("C", -1), ("S", 0), ("C", -1)),
    (("C", -1), ("C", 1), ("S", 0), ("C", 1)),
    (("C", -1), ("C", 1), ("S", 0), ("C", -1)),
    # CCSCC：更长的绕行解，用于极窄空间
    (("C", 1), ("C", -1), ("S", 0), ("C", 1), ("C", -1)),
    (("C", -1), ("C", 1), ("S", 0), ("C", -1), ("C", 1)),
    # 退化情形：直线、单弧
    (("S", 0), ("C", 1)),
    (("S", 0), ("C", -1)),
    (("C", 1), ("S", 0)),
    (("C", -1), ("S", 0)),
]


def _step(pose, kind, steer, length):
    """按单位半径积分一段，返回新位姿。"""
    x, y, th = pose
    if kind == "S" or abs(length) < 1e-12:
        return (x + length * math.cos(th), y + length * math.sin(th), th)
    th2 = th + steer * length
    if steer > 0:
        return (x - math.sin(th) + math.sin(th2),
                y + math.cos(th) - math.cos(th2), th2)
    return (x + math.sin(th) - math.sin(th2),
            y - math.cos(th) + math.cos(th2), th2)


def integrate(pattern, lengths, start=(0.0, 0.0, 0.0)):
    p = start
    for (kind, steer), ln in zip(pattern, lengths):
        p = _step(p, kind, steer, ln)
    return p


class RSPath:
    """一段 Reeds-Shepp 曲线（真实米制）。"""

    def __init__(self, pattern, lengths, radius, start_pose):
        self.pattern = pattern
        self.radius = float(radius)
        self.start_pose = tuple(start_pose)
        # 转成米制：圆弧段长度 = R * 角度，直线段长度保持
        self.segments = []
        for (kind, steer), ln in zip(pattern, lengths):
            self.segments.append((kind, steer, ln * self.radius))

    @property
    def length(self):
        return sum(abs(s[2]) for s in self.segments)

    @property
    def n_cusps(self):
        """换挡次数（相邻段前进/后退方向相反处）。"""
        dirs = [1 if s[2] >= 0 else -1 for s in self.segments if abs(s[2]) > 1e-6]
        return sum(1 for a, b in zip(dirs, dirs[1:]) if a != b)

    def _to_world(self, p_norm):
        """归一化位姿 -> 真实世界位姿（按起点位姿与转弯半径还原）。"""
        sx, sy, sth = self.start_pose
        px, py, pth = p_norm[0] * self.radius, p_norm[1] * self.radius, p_norm[2]
        c, s = math.cos(sth), math.sin(sth)
        return (sx + c * px - s * py, sy + s * px + c * py, wrap(sth + pth))

    def sample(self, step=0.15):
        """离散成 (x, y, yaw, direction, curvature) 序列（真实米制）。

        [同学 C 修复] **起点本身必须出现在路径里**。原实现「先积分一步、再记录」，
        第一条样点离起点一个 step（0.15~0.20 m）——跟车器从规划位姿出发时，
        投影被钳在 path[0] 上，凭空多出 step 大小的位置/航向误差。叠加
        strip_micro_head 从换挡点重规划（又一次缺起点）后，W9 的路径头部与
        交接位姿差 0.33 m / 11°，第一拍 φ=−14.3°、κ 直接顶死限幅，
        横向峰值 1.02 m（§6.23⑤2 的独立缺陷）。
        """
        segs = [s for s in self.segments if abs(s[2]) > 1e-6]
        out = []
        if segs:
            kind, steer, ln = segs[0]
            d0 = 1 if ln > 0 else -1
            c0 = 0.0 if kind == "S" else steer / self.radius
        else:
            d0, c0 = 1, 0.0
        sx, sy, sth = self._to_world((0.0, 0.0, 0.0))
        out.append((sx, sy, sth, d0, c0))
        p = (0.0, 0.0, 0.0)          # 归一化坐标下从原点积分
        for kind, steer, ln in self.segments:
            if abs(ln) < 1e-6:
                continue
            n = max(1, int(math.ceil(abs(ln) / step)))
            ds = ln / n               # 每步真实长度
            dnorm = ds / self.radius  # 归一化增量（圆弧=角度，直线=距离/R）
            curv = 0.0 if kind == "S" else steer / self.radius
            for _ in range(n):
                p = _step(p, kind, steer, dnorm)
                xr, yr, thr = self._to_world(p)
                out.append((xr, yr, thr, 1 if ds > 0 else -1, curv))
        return out

    def __repr__(self):
        seg = ", ".join(f"{k}{'' if s == 0 else ('+' if s > 0 else '-')}:{l:.2f}"
                        for k, s, l in self.segments if abs(l) > 1e-6)
        return f"RSPath(len={self.length:.2f}, cusps={self.n_cusps}, {seg})"


def _residual(lengths, pattern, goal_n):
    p = integrate(pattern, lengths)
    return np.array([p[0] - goal_n[0], p[1] - goal_n[1], wrap(p[2] - goal_n[2])])


def _initial_guesses(pattern, goal_n, rng, n_random=6):
    """给最小二乘一组初值：均匀分摊 + 若干随机扰动。"""
    k = len(pattern)
    dist = math.hypot(goal_n[0], goal_n[1])
    guesses = []
    base = []
    for kind, _ in pattern:
        if kind == "S":
            base.append(max(0.4, dist / max(1, sum(1 for kd, _ in pattern if kd == "S"))))
        else:
            base.append(0.5)
    guesses.append(np.array(base, dtype=float))
    guesses.append(-np.array(base, dtype=float))
    for _ in range(n_random):
        g = np.array(base, dtype=float) * (1.0 + 0.6 * rng.standard_normal(k))
        guesses.append(g)
    return guesses


def reeds_shepp(start_pose, goal_pose, radius, max_cusps=3, quick=False, rng=None,
                all_candidates=False):
    """求一条从 start 到 goal 的 Reeds-Shepp 曲线；无解返回 None。

    start_pose / goal_pose: (x, y, yaw)，单位 m / rad。
    radius: 最小转弯半径（m）。
    quick: True 时只尝试 CSC/CCC 与少量初值，用于搜索过程中的解析扩展。
    all_candidates: True 时返回**每个模式族的最优解**列表（按代价升序），
        供调用方按余量挑解。「最短」和「最安全」经常不是同一条线：
        实测 W5/W7/E3/E8 的最短解析解车体间隙只有 +0.068 m，而同起终点的
        别的模式族能给出 0.3 m 以上的解 —— 单返回最短解等于把选择权收走了。
    """
    rng = rng or np.random.default_rng(12345)
    sx, sy, sth = start_pose
    gx, gy, gth = goal_pose
    # 变换到以起点为原点、朝向为 +x 的归一化坐标
    dx, dy = gx - sx, gy - sy
    c, s = math.cos(-sth), math.sin(-sth)
    lx = (c * dx - s * dy) / radius
    ly = (s * dx + c * dy) / radius
    lth = wrap(gth - sth)

    patterns = PATTERNS[:6] if (quick and max_cusps <= 2) else PATTERNS
    n_random = 1 if quick else 6
    max_nfev = 120 if quick else 400

    best = None
    cands = []
    for pattern in patterns:
        if len(pattern) > max_cusps * 2 + 2:
            continue
        pat_best = None
        for g0 in _initial_guesses(pattern, (lx, ly), rng, n_random):
            try:
                sol = least_squares(_residual, g0, args=(pattern, (lx, ly, lth)),
                                    bounds=(-8.0, 8.0), xtol=1e-12, ftol=1e-12,
                                    gtol=1e-12, max_nfev=max_nfev)
            except Exception:
                continue
            if not sol.success:
                continue
            lengths = sol.x
            if np.max(np.abs(_residual(lengths, pattern, (lx, ly, lth)))) > 1e-6:
                continue
            path = RSPath(pattern, lengths, radius, start_pose)
            if path.length < 1e-3 or path.n_cusps > max_cusps:
                continue
            score = path.length + 2.0 * path.n_cusps
            if pat_best is None or score < pat_best[0] - 1e-9:
                pat_best = (score, path)
        if pat_best is None:
            continue
        cands.append(pat_best)
        if best is None or pat_best[0] < best[0] - 1e-9:
            best = pat_best
    if all_candidates:
        cands.sort(key=lambda t: t[0])
        return [p for _s, p in cands]
    return best[1] if best else None
