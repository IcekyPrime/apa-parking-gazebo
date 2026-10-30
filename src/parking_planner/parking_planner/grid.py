"""占据栅格：栅格化、膨胀、测地距离场（Hybrid A* 的 2D 启发式）。

所有半径类操作都通过欧氏距离变换实现（`scipy.ndimage.distance_transform_edt`），
避免手工卷积核在分辨率变化时失真。
"""

import heapq
import math

import numpy as np
from scipy import ndimage


class Grid:
    """二值占据栅格，1 = 占据，0 = 可通行。"""

    def __init__(self, xmin, ymin, res, width, height, data=None):
        self.xmin = float(xmin)
        self.ymin = float(ymin)
        self.res = float(res)
        self.width = int(width)
        self.height = int(height)
        self.data = np.zeros((self.height, self.width), dtype=np.uint8) \
            if data is None else data

    # ------------------------------------------------------------------ 构造
    @classmethod
    def from_bounds(cls, xmin, ymin, xmax, ymax, res):
        # 向外扩一圈边界，避免贴着栅格边缘的障碍被裁剪
        width = int(math.ceil((xmax - xmin) / res)) + 1
        height = int(math.ceil((ymax - ymin) / res)) + 1
        g = cls(xmin, ymin, res, width, height)
        # 最外圈标为占据，等价于「不可驶出地图」
        g.data[0, :] = 1
        g.data[-1, :] = 1
        g.data[:, 0] = 1
        g.data[:, -1] = 1
        return g

    @property
    def xmax(self):
        return self.xmin + (self.width - 1) * self.res

    @property
    def ymax(self):
        return self.ymin + (self.height - 1) * self.res

    def copy(self):
        return Grid(self.xmin, self.ymin, self.res, self.width, self.height,
                    self.data.copy())

    # -------------------------------------------------------------- 坐标换算
    def to_index(self, x, y):
        ix = int(round((x - self.xmin) / self.res))
        iy = int(round((y - self.ymin) / self.res))
        return ix, iy

    def to_world(self, ix, iy):
        return (self.xmin + ix * self.res, self.ymin + iy * self.res)

    def inside(self, ix, iy):
        return 0 <= ix < self.width and 0 <= iy < self.height

    def occupied(self, x, y):
        ix, iy = self.to_index(x, y)
        if not self.inside(ix, iy):
            return True
        return bool(self.data[iy, ix])

    def free(self, x, y):
        return not self.occupied(x, y)

    # ------------------------------------------------------------------ 写入
    def fill_rect(self, rect, value=1):
        """把一个有向矩形（或差速扩张的矩形）栅格化。"""
        xs = [c[0] for c in rect.corners]
        ys = [c[1] for c in rect.corners]
        ix0, iy0 = self.to_index(min(xs) - self.res, min(ys) - self.res)
        ix1, iy1 = self.to_index(max(xs) + self.res, max(ys) + self.res)
        ix0, iy0 = max(0, ix0), max(0, iy0)
        ix1, iy1 = min(self.width - 1, ix1), min(self.height - 1, iy1)
        if ix1 < ix0 or iy1 < iy0:
            return
        c, s = math.cos(rect.yaw), math.sin(rect.yaw)
        hl, hw = rect.hl + 0.5 * self.res, rect.hw + 0.5 * self.res
        for iy in range(iy0, iy1 + 1):
            wy = self.ymin + iy * self.res - rect.cy
            for ix in range(ix0, ix1 + 1):
                wx = self.xmin + ix * self.res - rect.cx
                lx = c * wx + s * wy
                ly = -s * wx + c * wy
                if abs(lx) <= hl and abs(ly) <= hw:
                    if self.data[iy, ix] == 0:
                        self.data[iy, ix] = value

    def fill_circle(self, cx, cy, r, value=1):
        ix0, iy0 = self.to_index(cx - r, cy - r)
        ix1, iy1 = self.to_index(cx + r, cy + r)
        ix0, iy0 = max(0, ix0), max(0, iy0)
        ix1, iy1 = min(self.width - 1, ix1), min(self.height - 1, iy1)
        r2 = r * r
        for iy in range(iy0, iy1 + 1):
            dy = self.ymin + iy * self.res - cy
            for ix in range(ix0, ix1 + 1):
                dx = self.xmin + ix * self.res - cx
                if dx * dx + dy * dy <= r2 and self.data[iy, ix] == 0:
                    self.data[iy, ix] = value

    def add_points(self, points, value=1):
        """把点云（map 帧）标为占据。"""
        if not points:
            return
        arr = np.asarray(points, dtype=float)
        ix = np.round((arr[:, 0] - self.xmin) / self.res).astype(int)
        iy = np.round((arr[:, 1] - self.ymin) / self.res).astype(int)
        ok = (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        self.data[iy[ok], ix[ok]] = value

    # ------------------------------------------------------------------ 形态学
    def inflated(self, radius):
        """按欧氏距离膨胀：任何与障碍物距离 <= radius 的格子都变为占据。"""
        if radius <= 0.0:
            return self.copy()
        free = self.data == 0
        # 到最近占据格的距离（米）
        dist = ndimage.distance_transform_edt(free) * self.res
        out = self.copy()
        out.data = np.where(dist <= radius, np.uint8(1), np.uint8(0)).astype(np.uint8)
        return out

    def distance_field(self, goal_xy):
        """从目标点出发的可通行测地距离（米），用作 2D 启发式。

        8 邻域 Dijkstra，斜向代价 sqrt(2)。不可通行处为 +inf。

        注意：内部必须用 float64。float32 存储会让 `nd < field - 1e-6` 这类
        比较在累积误差下误判，导致搜索在距离 ~0.8 m 处提前终止（实测踩过）。
        """
        field = np.full((self.height, self.width), np.inf, dtype=np.float64)
        gx, gy = self.to_index(goal_xy[0], goal_xy[1])
        if not self.inside(gx, gy):
            return field
        if self.data[gy, gx] != 0:
            # 目标落在占据格：找最近的可通行格
            gx, gy = self._nearest_free(gx, gy)

        field[gy, gx] = 0.0
        h, w = self.height, self.width
        data = self.data
        heap = [(0.0, gx, gy)]
        res = self.res
        neigh = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
                 (1, 1, 1.41421356), (1, -1, 1.41421356),
                 (-1, 1, 1.41421356), (-1, -1, 1.41421356))
        while heap:
            d, ix, iy = heapq.heappop(heap)
            if d > field[iy, ix]:
                continue
            for dx, dy, cost in neigh:
                nx, ny = ix + dx, iy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                if data[ny, nx] != 0:
                    continue
                nd = d + cost * res
                if nd < field[ny, nx] - 1e-6:
                    field[ny, nx] = nd
                    heapq.heappush(heap, (nd, nx, ny))
        return field

    def _nearest_free(self, ix, iy, max_r=20):
        for r in range(1, max_r):
            for dx in range(-r, r + 1):
                for dy in (-r, r):
                    nx, ny = ix + dx, iy + dy
                    if self.inside(nx, ny) and self.data[ny, nx] == 0:
                        return nx, ny
            for dy in range(-r + 1, r):
                for dx in (-r, r):
                    nx, ny = ix + dx, iy + dy
                    if self.inside(nx, ny) and self.data[ny, nx] == 0:
                        return nx, ny
        return ix, iy

    def sample_field(self, field, x, y):
        """双线性采样距离场；落在栅格外或不可达返回 +inf。"""
        fx = (x - self.xmin) / self.res
        fy = (y - self.ymin) / self.res
        ix, iy = int(math.floor(fx)), int(math.floor(fy))
        if ix < 0 or iy < 0 or ix >= self.width - 1 or iy >= self.height - 1:
            return float("inf")
        tx, ty = fx - ix, fy - iy
        # 先把 inf 截断成一个大数，避免 inf * 0 产生 nan
        big = 1.0e4
        v00 = min(float(field[iy, ix]), big)
        v10 = min(float(field[iy, ix + 1]), big)
        v01 = min(float(field[iy + 1, ix]), big)
        v11 = min(float(field[iy + 1, ix + 1]), big)
        v = (v00 * (1 - tx) * (1 - ty) + v10 * tx * (1 - ty)
             + v01 * (1 - tx) * ty + v11 * tx * ty)
        if v >= big * 0.5:
            return float("inf")
        return float(v)

    def stats(self):
        free = int(np.count_nonzero(self.data == 0))
        return {"width": self.width, "height": self.height,
                "res": self.res, "free": free,
                "occ": self.width * self.height - free}
