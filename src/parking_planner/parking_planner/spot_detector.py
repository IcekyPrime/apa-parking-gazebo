"""空闲车位检测：用激光雷达的「自由空间穿透」证据判断车位是否被占用。

思路
----
2D 激光雷达看不到地面的车位漆线，因此不靠「识别线」而靠**空间可穿越性**：
从雷达原点沿每条射线把格子标为（a）命中点=有障碍，（b）命中前的格子=自由。
落在车位内部、且被标记为「自由」的格子比例高，说明车辆能开进去 → 空闲；
若车位里停着车，射线会打在车身前脸上，车位内部格子既不被命中也不被自由
穿透 → 自由比例低。

这与课件 8 的占据栅格（对数几率更新）同源，这里用计数衰减代替 log-odds，
便于调参和可视化。
"""

import math

import numpy as np

from .geometry import Rect


class SpotDetector:
    def __init__(self, layout, res=0.10, max_range=18.0,
                 decay_period=3.0, decay=0.85, ray_stride=2, step=0.20,
                 occ_thresh=0.5, free_ratio_min=0.40):
        self.layout = layout
        self.res = float(res)
        self.max_range = float(max_range)
        self.decay_period = float(decay_period)
        self.decay = float(decay)
        self.ray_stride = int(ray_stride)
        self.step = float(step)
        self.occ_thresh = float(occ_thresh)
        self.free_ratio_min = float(free_ratio_min)

        xmin, xmax, ymin, ymax = layout.roi if layout.roi else layout.bounds()
        self.xmin, self.ymin = xmin, ymin
        self.width = int(math.ceil((xmax - xmin) / self.res)) + 1
        self.height = int(math.ceil((ymax - ymin) / self.res)) + 1
        self.hits = np.zeros((self.height, self.width), dtype=np.float32)
        self.free = np.zeros((self.height, self.width), dtype=bool)
        self._since_decay = 0.0
        self._n_scans = 0
        self.last_pose = None

    # ------------------------------------------------------------------ 内部工具
    def _index(self, x, y):
        ix = int(round((x - self.xmin) / self.res))
        iy = int(round((y - self.ymin) / self.res))
        if 0 <= ix < self.width and 0 <= iy < self.height:
            return ix, iy
        return None

    def reset(self):
        self.hits[:] = 0.0
        self.free[:] = False
        self._n_scans = 0

    # -------------------------------------------------------------------- 输入
    def add_scan(self, ranges, angle_min, angle_increment, pose, dt=0.1,
                 lidar_offset=(-0.15, 0.0)):
        """把一帧 LaserScan 按自车位姿投到地图帧，更新命中/自由栅格。

        两类射线都要用：
        * 有回波：命中点标占据，命中前沿途标自由；
        * 无回波（inf 或超出量程）：说明该方向到量程内没有障碍，整段标自由。
          车位内部正是靠这一类射线才被覆盖 —— 若只处理有回波的射线，空车位
          永远得不到「自由」证据（实测踩过这个坑）。
        """
        x, y, th = pose
        self.last_pose = (x, y, th)
        c, s = math.cos(th), math.sin(th)
        ox, oy = lidar_offset
        lx = x + c * ox - s * oy
        ly = y + s * ox + c * oy

        arr = np.asarray(ranges, dtype=np.float64)
        n = arr.size
        if n == 0:
            return
        angle = angle_min + np.arange(n, dtype=np.float64) * angle_increment

        self._n_scans += 1
        self._since_decay += dt
        if self._since_decay >= self.decay_period:
            self.hits *= self.decay
            self._since_decay = 0.0

        keep = np.arange(0, n, self.ray_stride)
        valid = np.isfinite(arr) & (arr > 0.25)
        hit_ray = valid & (arr <= self.max_range - 0.05)

        # ---- 自由空间：沿射线采样（向量化） ----
        dists = np.arange(0.35, self.max_range, self.step)
        a = angle[keep][:, None] + th
        px = lx + np.cos(a) * dists[None, :]
        py = ly + np.sin(a) * dists[None, :]
        # 有回波的射线只标到命中点前；无回波的整段标自由
        limits = np.where(hit_ray[keep], np.maximum(0.0, arr[keep] - 0.4),
                          self.max_range)[:, None]
        mask = dists[None, :] < limits
        ix = np.round((px - self.xmin) / self.res).astype(np.int32)
        iy = np.round((py - self.ymin) / self.res).astype(np.int32)
        ok = mask & (ix >= 0) & (ix < self.width) & (iy >= 0) & (iy < self.height)
        self.free[iy[ok], ix[ok]] = True

        # ---- 命中点 ----
        hx = lx + np.cos(angle[hit_ray] + th) * arr[hit_ray]
        hy = ly + np.sin(angle[hit_ray] + th) * arr[hit_ray]
        jx = np.round((hx - self.xmin) / self.res).astype(np.int32)
        jy = np.round((hy - self.ymin) / self.res).astype(np.int32)
        ok2 = (jx >= 0) & (jx < self.width) & (jy >= 0) & (jy < self.height)
        if np.any(ok2):
            jx, jy = jx[ok2], jy[ok2]
            np.add.at(self.hits, (jy, jx), 1.0)
            self.free[jy, jx] = False

    # -------------------------------------------------------------------- 评估
    def probe_segment(self, stall, depth_in=1.2, step=0.15):
        """沿「自车 → 车位入口内侧」的探测线检查是否被占据。

        返回 (state, info)：
        * 'occupied'：探测线在车位内部先撞到障碍（车位里有车）；
        * 'blocked' ：探测线在车位外被挡住（看不见，无法判断）；
        * 'free'    ：整条探测线没有任何障碍。
        """
        if self.last_pose is None:
            return 'blocked', {'hit_at': None}
        ex, ey, _ = self.last_pose
        # 入口在内侧方向上、距车位中心 (l/2 - depth_in) 处
        depth_dir = (math.cos(stall.yaw), math.sin(stall.yaw))
        px = stall.cx - depth_dir[0] * (stall.l * 0.5 - depth_in)
        py = stall.cy - depth_dir[1] * (stall.l * 0.5 - depth_in)

        seg = math.hypot(px - ex, py - ey)
        if seg < 0.3 or seg > self.max_range:
            return 'blocked', {'hit_at': None, 'reason': 'out_of_range'}

        n = max(2, int(seg / step))
        rect = stall.rect
        for i in range(1, n + 1):
            t = i / n
            x = ex + (px - ex) * t
            y = ey + (py - ey) * t
            cell = self._index(x, y)
            if cell is None:
                continue
            if self.hits[cell[1], cell[0]] >= self.occ_thresh:
                inside = rect.contains(x, y, margin=0.35)
                return ('occupied' if inside else 'blocked'), {'hit_at': (x, y)}
        return 'free', {'hit_at': None}

    def evaluate_stall(self, stall, shrink=0.35):
        rect = stall.inner_rect(shrink)
        xs = [p[0] for p in rect.corners]
        ys = [p[1] for p in rect.corners]
        ix0 = int(round((min(xs) - self.xmin) / self.res))
        ix1 = int(round((max(xs) - self.xmin) / self.res))
        iy0 = int(round((min(ys) - self.ymin) / self.res))
        iy1 = int(round((max(ys) - self.ymin) / self.res))
        ix0, iy0 = max(0, ix0), max(0, iy0)
        ix1, iy1 = min(self.width - 1, ix1), min(self.height - 1, iy1)
        total = occ = free = 0
        if ix1 > ix0 and iy1 > iy0:
            c, s = math.cos(rect.yaw), math.sin(rect.yaw)
            for iy in range(iy0, iy1 + 1):
                wy = self.ymin + iy * self.res - rect.cy
                for ix in range(ix0, ix1 + 1):
                    wx = self.xmin + ix * self.res - rect.cx
                    lx = c * wx + s * wy
                    ly = -s * wx + c * wy
                    if abs(lx) > rect.hl or abs(ly) > rect.hw:
                        continue
                    total += 1
                    if self.hits[iy, ix] >= self.occ_thresh:
                        occ += 1
                    elif self.free[iy, ix]:
                        free += 1
        occ_ratio = occ / total if total else 1.0
        free_ratio = free / total if total else 0.0

        state, extra = self.probe_segment(stall)
        return {"occ_ratio": occ_ratio, "free_ratio": free_ratio, "total": total,
                "state": state,
                "is_free": state == 'free',
                "hit_at": extra.get('hit_at')}

    def evaluate_all(self, shrink=0.35):
        out = {}
        for stall in self.layout.stalls:
            out[stall.id] = self.evaluate_stall(stall, shrink)
        return out

    # -------------------------------------------------------------------- 选位
    def select(self, pose, exclude=(), max_dist=120.0, min_dist=5.0, prefer=None,
               heading_penalty=6.0, runup_min=6.5, region=None):
        """挑一个空闲、可用的车位。

        评分 = 距离 + 朝向惩罚 + 助跑惩罚 + 类型惩罚：
        * 垂直车位一把方向就能进，优先；
        * 平行车位需要多点揉库，加 8 分惩罚，作为次选（显式指定车位时不受影响）。
        """
        x, y, th = pose
        cands = []
        for stall in self.layout.stalls:
            if stall.id in exclude:
                continue
            if region and stall.region != region:
                continue
            d = math.hypot(stall.cx - x, stall.cy - y)
            if d < min_dist or d > max_dist:
                continue
            info = self.evaluate_stall(stall)
            if not info["is_free"]:
                continue
            bearing = math.atan2(stall.cy - y, stall.cx - x)
            ahead = math.cos(bearing - th)
            score = d + heading_penalty * (1.0 - ahead) * 0.5
            if stall.kind == "parallel":
                score += 8.0
            # 「助跑距离」只对垂直车位有意义（平行车位是侧方倒入）
            if stall.kind == "perpendicular":
                proj = (stall.cx - x) * math.cos(th) + (stall.cy - y) * math.sin(th)
                if proj < runup_min:
                    score += 40.0 + 4.0 * (runup_min - proj)
            if prefer and stall.id in prefer:
                score -= 100.0 + prefer.index(stall.id)
            cands.append((score, stall, info))
        if not cands:
            return None, None
        cands.sort(key=lambda t: t[0])
        return cands[0][1], cands[0][2]
