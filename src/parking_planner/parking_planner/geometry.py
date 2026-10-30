"""二维几何工具：位姿、角度环绕、矩形（OBB）与坐标变换。

规划层内部统一使用 map 坐标系（slam_toolbox 的 map 帧，原点在自车出生位姿，
初始朝向为 +x）。角度一律用弧度，位姿用 (x, y, yaw) 三元组。
"""

import math

TWO_PI = 2.0 * math.pi


def wrap(a):
    """把角度归一化到 (-pi, pi]。"""
    a = (a + math.pi) % TWO_PI - math.pi
    return a


def wrap_2pi(a):
    """把角度归一化到 [0, 2pi)。"""
    return a % TWO_PI


def pose_distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def pose_to_matrix(p):
    """位姿 -> 3x3 齐次矩阵（行主序展开的 9 元组）。"""
    c, s = math.cos(p[2]), math.sin(p[2])
    return (c, -s, p[0], s, c, p[1], 0.0, 0.0, 1.0)


def transform_point(p, x, y, yaw):
    """把 p 变换到以 (x, y, yaw) 为基准的坐标系。"""
    c, s = math.cos(yaw), math.sin(yaw)
    dx, dy = p[0] - x, p[1] - y
    return (c * dx + s * dy, -s * dx + c * dy)


def transform_pose(pose, x, y, yaw):
    """把 pose 变换到以 (x, y, yaw) 为基准的坐标系（含朝向）。"""
    px, py = transform_point(pose, x, y, yaw)
    return (px, py, wrap(pose[2] - yaw))


def compose(a, b):
    """位姿复合：a ⊕ b（在 a 的坐标系里执行 b）。"""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * b[0] - s * b[1],
            a[1] + s * b[0] + c * b[1],
            wrap(a[2] + b[2]))


class Rect:
    """有向矩形：中心 (cx, cy)，沿局部 x 的半长 hl、沿局部 y 的半宽 hw，朝向 yaw。

    yaw 定义局部 +x 轴的方向。用于障碍物、车位、车体轮廓。
    """

    __slots__ = ("cx", "cy", "yaw", "hl", "hw", "name")

    def __init__(self, cx, cy, yaw, hl, hw, name=""):
        self.cx = float(cx)
        self.cy = float(cy)
        self.yaw = float(yaw)
        self.hl = float(hl)
        self.hw = float(hw)
        self.name = name

    @property
    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        out = []
        for lx, ly in ((self.hl, self.hw), (-self.hl, self.hw),
                       (-self.hl, -self.hw), (self.hl, -self.hw)):
            out.append((self.cx + c * lx - s * ly, self.cy + s * lx + c * ly))
        return out

    def contains(self, x, y, margin=0.0):
        lx, ly = transform_point((x, y), self.cx, self.cy, self.yaw)
        return abs(lx) <= self.hl + margin and abs(ly) <= self.hw + margin

    def to_dict(self):
        d = {"cx": round(self.cx, 4), "cy": round(self.cy, 4),
             "yaw": round(self.yaw, 5), "hl": round(self.hl, 4),
             "hw": round(self.hw, 4)}
        if self.name:
            d["name"] = self.name
        return d

    def __repr__(self):
        return f"Rect({self.name}, c=({self.cx:.2f},{self.cy:.2f}), yaw={self.yaw:.3f}, hl={self.hl:.2f}, hw={self.hw:.2f})"


def circles_cover_rect(body, radius):
    """用若干圆覆盖有向矩形，返回圆心列表（用于栅格碰撞检查）。

    对细长车体（4.35 m x 1.85 m），用 3 个半径 >= 半宽的圆近似覆盖，
    既避免逐点测试的锯齿，又不会像单个外接圆那样过度保守。
    """
    if body.hl <= radius * 0.5:
        return [(body.cx, body.cy)]
    n = max(1, int(math.ceil(body.hl / (radius * 0.8))))
    c, s = math.cos(body.yaw), math.sin(body.yaw)
    out = []
    # 圆心沿长轴均匀分布，覆盖 [-hl + radius, hl - radius] 区间
    span = max(0.0, body.hl - radius)
    for i in range(n):
        t = 0.0 if n == 1 else (-span + 2.0 * span * i / (n - 1))
        out.append((body.cx + c * t, body.cy + s * t))
    return out


def point_rect_gap(px, py, rect):
    """点到有向矩形外廓的距离（矩形内为 0）。"""
    lon, lat = transform_point((px, py), rect.cx, rect.cy, rect.yaw)
    return math.hypot(max(0.0, abs(lon) - rect.hl),
                      max(0.0, abs(lat) - rect.hw))


def body_gap(pose, obstacles, half_l=2.175, half_w=0.925, n_edge=6):
    """车体矩形（真实矩形，不是近似圆）到最近障碍的**几何间隙**（m）。

    负值 = 车体与障碍已经重叠（真剐蹭），0 = 贴着。

    为什么需要它：规划器用的是 7 个半径 0.95 m 的圆做碰撞检查，报出来的
    `min_clearance` 是「圆心到障碍再减 0.95」，圆在车角处比真实车体保守、
    在长边处又略宽松，**不能直接当作「会不会剐」的结论**。要判断一次执行到底
    有没有蹭到，只能拿**真实车体矩形**和障碍矩形去量。

    采样方式：自车边界取 n_edge 等分的上下两条长边 + 左右两条短边，再把每个
    障碍的 4 个角点回代到自车矩形 —— 两个凸矩形之间的最近距离一定落在
    「某个角点 ↔ 另一条边」上，两侧都取到就足够精确（误差在 1 cm 量级）。
    """
    if not obstacles:
        return float('inf')
    x, y, th = pose
    c, s = math.cos(th), math.sin(th)
    pts = []
    for i in range(n_edge + 1):
        u = -half_l + 2.0 * half_l * i / n_edge
        pts.append((u, -half_w))
        pts.append((u, half_w))
    for i in range(1, n_edge):
        v = -half_w + 2.0 * half_w * i / n_edge
        pts.append((-half_l, v))
        pts.append((half_l, v))
    best = float('inf')
    reach = math.hypot(half_l, half_w) + 1.0
    for ob in obstacles:
        if math.hypot(ob.cx - x, ob.cy - y) > reach + math.hypot(ob.hl, ob.hw):
            continue
        for (lx, ly) in pts:
            wx = x + c * lx - s * ly
            wy = y + s * lx + c * ly
            g = point_rect_gap(wx, wy, ob)
            if g < best:
                best = g
                if best <= -1.0:
                    return best
        for (ocx, ocy) in ob.corners:
            dx, dy = ocx - x, ocy - y
            lon = c * dx + s * dy
            lat = -s * dx + c * dy
            g = math.hypot(max(0.0, abs(lon) - half_l),
                           max(0.0, abs(lat) - half_w))
            if g < best:
                best = g
    return best

