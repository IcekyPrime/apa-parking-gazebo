"""停车场场景布局：从 Gazebo world 解析车位与障碍，并生成占据栅格。

坐标系
------
解析结果一律保存在 **world 帧**（Gazebo 世界坐标）。运行期由 `to_frame(spawn)`
一次性变换到 **map 帧**（slam_toolbox 建立的地图帧，原点在自车生成位姿）。这样做
是因为自车可以在主路任意位置生成，map 帧原点随生成位姿变化，布局不能写死。

车位类型与预备位姿
------------------
* 垂直车位（perpendicular）：车头朝里，预备位姿在车位正前方 1.6 m、朝向车位；
* 平行车位（parallel）：车身与车道平行，预备位姿在车位前缘外侧、车身摆正，
  之后由 Hybrid A* 做多点揉库倒入（倒车换挡由规划器决定）。
"""

import math
import os
import xml.etree.ElementTree as ET

from .geometry import Rect, transform_pose, wrap

# ---------------------------------------------------------------- 预备位姿表
# 垂直车位的预备位姿改成查表（见 `Stall.staging_pose` 的说明）。
# 表由 `scripts/compute_staging.py` 生成，默认放在包的 config 目录下。
STAGING_OVERRIDES = {}
# 注意：`--symlink-install` 下 `__file__` 指向 install 目录里的**符号链接**，
# `abspath` 不会解析它，会得到一个不存在的 `.../site-packages/config`。
# 所以这里先 realpath 再拼，另外留两个兜底路径（源码树 / 家目录）。
_STAGING_CANDIDATES = [
    os.path.join(os.path.dirname(os.path.dirname(
        os.path.realpath(__file__))), 'config', 'staging_poses.yaml'),
    os.path.join(os.path.dirname(os.path.dirname(
        os.path.abspath(__file__))), 'config', 'staging_poses.yaml'),
    os.path.expanduser('~/apa_ws/src/parking_planner/config/staging_poses.yaml'),
]
_STAGING_YAML = _STAGING_CANDIDATES[0]
_STAGING_LOADED = False


def load_staging_overrides(path=None, force=False):
    """读 `staging_poses.yaml` 里的 `staging: {车位id: [x, y, yaw]}`。"""
    global _STAGING_LOADED
    if _STAGING_LOADED and not force and path is None:
        return STAGING_OVERRIDES
    _STAGING_LOADED = True
    cands = [path] if path else _STAGING_CANDIDATES
    p = next((c for c in cands if c and os.path.exists(c)), None)
    if p is None:
        return STAGING_OVERRIDES
    try:
        import yaml
        with open(p, encoding='utf-8') as f:
            data = yaml.safe_load(f) or {}
    except Exception:                                  # noqa: BLE001
        return STAGING_OVERRIDES
    for k, v in (data.get('staging') or {}).items():
        if isinstance(v, (list, tuple)) and len(v) >= 3:
            STAGING_OVERRIDES[str(k)] = (float(v[0]), float(v[1]), float(v[2]))
    return STAGING_OVERRIDES


# 导入即加载：这样无论调用方走 `from_world` 还是 `from_yaml`，`staging_pose`
# 都能拿到表里的位姿（之前挂在 `from_world` 末尾，`from_yaml` 那条路径漏了）。
load_staging_overrides()


# 车型参数（与 parking_lot_city/models/ego_car/model.sdf 保持一致）
WHEELBASE = 2.70
LENGTH = 4.35
WIDTH = 1.85
MAX_STEER = 0.61

# 实测标定（见 docs/03 开环转向实验）：满舵实际前轮转角约 0.49 rad，
# 规划必须按等效转角算转弯半径，否则车辆走不出规划的圆弧。
EFFECTIVE_MAX_STEER = 0.44
MIN_TURN_RADIUS = WHEELBASE / math.tan(EFFECTIVE_MAX_STEER)   # ≈ 5.74 m
STEER_CMD_GAIN = 1.40

# [同学 C] 跟踪器（下发 ω 那一侧）的舵角限幅，必须**大于**规划用的
# EFFECTIVE_MAX_STEER，否则在最大曲率段上闭环没有修正余量。
# 台架实测：指令 δ=0.49 rad 时车实际给出 κ≈0.175（R≈5.72 m），
# 满舵（插件限 0.61）时实际只能到 κ≈0.2015（R≈4.96 m）。
# 取 0.50 → 指令 κ 上限 0.202，正好等于车真正能做到的上限，
# 相对规划路径的最大曲率 0.1747 留出约 16% 的反馈余量。
FOLLOW_MAX_STEER = 0.50

INCLUDE_FOOTPRINTS = {
    "parked_car": (4.35, 1.85),
    "city_car": (4.35, 1.85),
    "city_car_b": (4.35, 1.85),
    "tree": (0.7, 0.7),
    "ego_car": (LENGTH, WIDTH),
}

# 路面高度阈值：collision 顶面低于该值视为可通行（路面、车位漆线）
PASSABLE_TOP_Z = 0.08


def _floats(text, n=6):
    vals = [float(v) for v in (text or "").split()]
    while len(vals) < n:
        vals.append(0.0)
    return vals


def _rect_in_box(rect, box, margin=0.0):
    xs = [c[0] for c in rect.corners]
    ys = [c[1] for c in rect.corners]
    xmin, xmax, ymin, ymax = box
    return (max(xs) >= xmin - margin and min(xs) <= xmax + margin
            and max(ys) >= ymin - margin and min(ys) <= ymax + margin)


class Stall:
    """一个车位（world 帧）。

    (cx, cy, yaw) 是停好后的目标位姿；`l` 沿车头方向，`w` 横向宽度。
    `lane_normal` 指向车道一侧的单位向量，用于计算预备位姿。
    """

    def __init__(self, sid, kind, cx, cy, yaw, l, w, lane_normal=(0.0, 1.0),
                 region="lot"):
        self.id = sid
        self.kind = kind
        self.cx = float(cx)
        self.cy = float(cy)
        self.yaw = float(yaw)
        self.l = float(l)
        self.w = float(w)
        self.lane_normal = (float(lane_normal[0]), float(lane_normal[1]))
        self.region = region
        # [同学 C 本轮] 在线算出来的预备位姿就存这儿（见 staging_pose）。
        # 不写回 yaml：一次运行里场上的占用情况可能变，位姿应该跟着变。
        self._staging = None

    def set_staging(self, pose):
        """覆盖预备位姿（由 `staging_search.search_staging` 在线算出）。"""
        self._staging = (float(pose[0]), float(pose[1]), float(pose[2]))

    @property
    def goal_pose(self):
        return (self.cx, self.cy, self.yaw)

    @property
    def rect(self):
        return Rect(self.cx, self.cy, self.yaw, self.l * 0.5, self.w * 0.5, self.id)

    def inner_rect(self, shrink=0.35):
        return Rect(self.cx, self.cy, self.yaw,
                    max(0.3, self.l * 0.5 - shrink),
                    max(0.2, self.w * 0.5 - shrink), self.id)

    @property
    def staging_pose(self):
        """泊车机动的起始位姿（预备位姿）。

        [同学 C 重构] 垂直车位的预备位姿**不再用几何公式硬算**，改成查
        `config/staging_poses.yaml` —— 那张表由 `scripts/compute_staging.py`
        生成：对每个车位枚举「90° 圆弧 + 沿轴直线」的候选起点，用真实
        Hybrid A* 逐个验证，只保留 **0 换挡且车体余量 ≥ 0.30 m** 的解。

        旧公式把预备位姿放在**车位正前方轴线上**（`x = x_车位 + l/2 + 1.6~2.6`），
        从那里出发必须先原地转 90° 才能进库；而最小转弯半径 6.39 m 在
        12.5 m 宽的过道里转不开，必然揉 2~3 把 —— 实测就是「永远不能一把入位、
        第一把倒在两个车位中间、擦着邻车右前角」。新表里的位姿在车位**前一个
        转弯半径**的过道点上，一段圆弧加一点直线就能进去，且它本身就落在
        过道中心线附近，路线天然把车带到那里。

        表里没有的车位（平行位等）仍走原来的几何公式。
        """
        key = str(self.id)
        if self._staging is not None:
            return self._staging        # 在线搜索结果（见 staging_search）
        if key in STAGING_OVERRIDES:
            # 表里存的是**车位局部坐标**偏移 [along, lateral, dyaw]，
            # 不是绝对坐标 —— 绝对坐标会在 `to_frame(spawn)` 之后混帧
            # （实测自车在 map 帧 (0,0)、预备位姿却是 world 帧 (-8,-16)，
            # 于是「自车-预备」算出 18 m，整轮无法规划）。
            along, lat, dyaw = STAGING_OVERRIDES[key]
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            return (self.cx + c * along - s * lat,
                    self.cy + s * along + c * lat,
                    wrap(self.yaw + dyaw))
        dx, dy = math.cos(self.yaw), math.sin(self.yaw)
        if self.kind == "perpendicular":
            # 倒车入库：车头朝车道（yaw 已指向车道），预备位姿在车位中心
            # **沿车头方向**外侧 = 车道一侧，且朝向与目标一致。
            # 于是「预备位姿 → 车位」就是一段直线倒车（s 米），只要路线能把
            # 车带到这个位姿，就是一把入位。旧写法 `-dx*s` 对应正向驶入。
            #
            # [同学 C 修复] 外移量 1.6 → **2.6** m（l=5.8 时 s=5.5 m，
            # 预备位姿 x=-11.5，车尾 x=-13.675）。
            # 原来 1.6 m：车尾在 x=-14.675，而车位口在 x=-14.1 —— **车尾已经
            # 探进车位 0.575 m**，等于「还没开始倒就先半个身子进库了」，
            # 手上没有余量修横向/航向残差，稍一跑偏整台车就错位到隔壁车位
            # （真机 W5 那次倒完落在了 W4 正前方）。
            # 现在车尾在车位口外 0.425 m，整台车都在过道里，倒车段 5.5 m
            # 是一条直线，横向残差可以在这段里慢慢收。
            # 取值是量出来的（`PLAN_CLEARANCE=0.15 verify_park_sim.py`，
            # 11 个可用垂直位全部 ≥0.35 m 达标、0 剐蹭）：
            #   1.6 → W9 执行间隙 +0.60 m，但车尾在车位口内
            #   2.0 → W9 +0.17 m（2 换挡）
            #   2.6 → W9 **+0.48 m**（2 换挡）   ← 取这个
            #   3.0 → W9 +0.17 m（2 换挡）
            # W4/W5 在四种取值下都是 9.1 m / 0 换挡 / +0.41 m，不受影响。
            s = self.l * 0.5 + 2.6
            return (self.cx + dx * s, self.cy + dy * s, self.yaw)
        # 平行车位：车身沿车位轴线，停在车位前缘外侧并靠近车道
        # 横向偏移取 w/2 + 1.5：车位进深 2.4 m（内边界离中心线 1.2 m），
        # 车半宽 0.925 m，所以 2.7 m 的偏移下，车身内侧离旁边停着的车还有
        # 约 0.58 m。原来用 w/2 + 1.0 = 2.2 m，只剩 7 cm，规划器加上栅格膨胀
        # 直接判成「起点被占据」。
        s = self.l * 0.5 + 2.2
        off = self.w * 0.5 + 1.5
        return (self.cx + dx * s + self.lane_normal[0] * off,
                self.cy + dy * s + self.lane_normal[1] * off,
                self.yaw)

    def transformed(self, spawn):
        x, y, yaw = transform_pose((self.cx, self.cy, self.yaw), *spawn)
        nx, ny = self.lane_normal
        c, s = math.cos(spawn[2]), math.sin(spawn[2])
        # 法向量只旋转，不平移
        ln = (c * nx + s * ny, -s * nx + c * ny)
        return Stall(self.id, self.kind, x, y, yaw, self.l, self.w, ln, self.region)

    def to_dict(self):
        return {"id": self.id, "kind": self.kind, "cx": round(self.cx, 4),
                "cy": round(self.cy, 4), "yaw": round(self.yaw, 5),
                "l": round(self.l, 3), "w": round(self.w, 3),
                "lane_normal": [round(self.lane_normal[0], 3),
                                round(self.lane_normal[1], 3)],
                "region": self.region}

    @classmethod
    def from_dict(cls, d):
        return cls(d["id"], d["kind"], d["cx"], d["cy"], d["yaw"], d["l"], d["w"],
                   d.get("lane_normal", (0.0, 1.0)), d.get("region", "lot"))

    def __repr__(self):
        return (f"Stall({self.id}, {self.kind}, {self.region}, "
                f"c=({self.cx:.2f},{self.cy:.2f}), yaw={self.yaw:.2f})")


class Layout:
    def __init__(self, obstacles=None, stalls=None, meta=None, resolution=0.10,
                 roi=None):
        self.obstacles = obstacles or []
        self.stalls = stalls or []
        self.meta = meta or {}
        self.resolution = float(resolution)
        self.roi = roi          # (xmin, xmax, ymin, ymax)，world 帧

    # ---------------------------------------------------------------- 解析 world
    @classmethod
    def from_world(cls, world_path, resolution=0.10):
        root = ET.parse(world_path).getroot()
        world = root.find("world") if root.tag == "sdf" else root

        raw_obs = []
        stall_pts = {"west": [], "east": []}
        para_x, para_y, para_w = [], None, 2.4
        back_x, back_y, back_w = [], None, 2.4
        side_pts = {"W": [], "E": []}
        side_w = {"W": 2.4, "E": 2.4}
        ego_pose = None

        for inc in world.findall("include"):
            uri = (inc.findtext("uri") or "").split("://")[-1].strip("/")
            name = inc.findtext("name") or uri
            px, py, _pz, _, _, pyaw = _floats(inc.findtext("pose"))
            if uri in ("ego_car", "ego_vehicle") or name == "ego_vehicle":
                ego_pose = (px, py, pyaw)
                continue
            # 动态干扰车（city_car / city_car_b）按初始位姿烘进栅格同样是错的，
            # 见下面 DYNAMIC_PREFIXES 处的说明。
            if name.startswith("traffic_car"):
                continue
            fp = INCLUDE_FOOTPRINTS.get(uri)
            if fp:
                raw_obs.append(Rect(px, py, pyaw, fp[0] * 0.5, fp[1] * 0.5, name))

        # 动态干扰物（行人、社会车辆）不能进静态占用栅格：
        # 它们在场景里是持续移动的，真实位置由激光 + 风险层在线处理；
        # 一旦按「初始位姿」烘进栅格，就会在自车过道上留下一个永不消失的障碍块，
        # 表现为「start 位姿被占据（车体与障碍重叠）」或「goal 与 start 在 2D
        # 栅格上不连通」——这两种报错都在把行人初始点放到场内过道后实测出现过。
        DYNAMIC_PREFIXES = ("pedestrian_", "traffic_car")

        for model in world.findall("model"):
            name = model.get("name") or ""
            if name.startswith(DYNAMIC_PREFIXES):
                continue
            px, py, pz, _, _, pyaw = _floats(model.findtext("pose"))

            if name.startswith("stall_line_"):
                size = _floats(model.findtext(".//box/size") or "0 0 0", 3)
                if size[0] > size[1]:
                    stall_pts["west" if px < 0 else "east"].append((px, py))
                continue
            if name.startswith("side_stall_W"):
                size = _floats(model.findtext(".//box/size") or "0 0 0", 3)
                if size[0] > size[1]:      # 横向分隔线（深×0.12）
                    side_pts["W"].append((px, py))
                    side_w["W"] = size[0]
                continue
            if name.startswith("side_stall_E"):
                size = _floats(model.findtext(".//box/size") or "0 0 0", 3)
                if size[0] > size[1]:
                    side_pts["E"].append((px, py))
                    side_w["E"] = size[0]
                continue
            if name.startswith("para_line_"):
                # [本轮] 分隔线尺寸 0.12×深度：深度从 world 读，不再硬编码 2.4
                size = _floats(model.findtext(".//box/size") or "0 0 0", 3)
                para_x.append(px)
                para_y = py if para_y is None else para_y
                if size[1] > 0.0:
                    para_w = size[1]
                continue
            if name == "para_edge":
                continue
            if name.startswith("back_stall_"):
                size = _floats(model.findtext(".//box/size") or "0 0 0", 3)
                if size[0] < size[1]:
                    back_x.append(px)
                    back_y = py if back_y is None else back_y
                    if size[1] > 0.0:
                        back_w = size[1]
                continue
            if name.startswith("back_stall_edge"):
                continue

            coll = model.find(".//collision")
            if coll is None:
                continue
            size = coll.findtext(".//box/size")
            if size is None:
                cyl = coll.find(".//cylinder")
                if cyl is not None:
                    r = float(cyl.findtext("radius") or 0.0)
                    h = float(cyl.findtext("length") or 0.0)
                    if pz + h * 0.5 > PASSABLE_TOP_Z:
                        raw_obs.append(Rect(px, py, pyaw, r, r, name))
                continue
            sx, sy, sz = _floats(size, 3)
            if pz + sz * 0.5 <= PASSABLE_TOP_Z or sx <= 0.0 or sy <= 0.0:
                continue
            raw_obs.append(Rect(px, py, pyaw, sx * 0.5, sy * 0.5, name))

        obstacles = list(raw_obs)
        stalls = []

        # ---- 场内垂直车位（**倒车入库**：车头朝车道）----
        # 车道在东西两侧过道（x=±8）：西排（x≈-17）车道在 +x 侧、东排（x≈+17）
        # 车道在 -x 侧。真实 APA 的垂直车位几乎都是倒车入库 —— 车尾先进位，
        # 停好后车头朝外（面向车道），出库时直接往前开走。
        # 旧写法 nose=π（西排）/0（东排）是「车头朝里」的正向入库，用户实测看到
        # 的就是「车头位于内侧、不是倒车入库」。
        # 配套改动见 staging_pose：偏移方向从 `-dx*s` 改成 `+dx*s`，
        # 这样预备位姿仍在车道一侧，且朝向与目标一致 → 入位就是一段直线倒车。
        for key, pts in stall_pts.items():
            if len(pts) < 2:
                continue
            x = sum(p[0] for p in pts) / len(pts)
            ys = sorted(p[1] for p in pts)
            nose = 0.0 if key == "west" else math.pi
            lane_n = (math.cos(nose), math.sin(nose))
            prefix = "W" if key == "west" else "E"
            for k in range(len(ys) - 1):
                cy = 0.5 * (ys[k] + ys[k + 1])
                stalls.append(Stall(f"{prefix}{k}", "perpendicular", x, cy, nose,
                                    5.8, ys[k + 1] - ys[k], lane_n, "lot"))

        # ---- 场内平行车位（南侧通道边，车道在 +y）----
        if len(para_x) >= 2 and para_y is not None:
            xs = sorted(para_x)
            for k in range(len(xs) - 1):
                stalls.append(Stall(f"P{k}", "parallel", 0.5 * (xs[k] + xs[k + 1]),
                                    para_y, 0.0, xs[k + 1] - xs[k], para_w,
                                    (0.0, 1.0), "lot"))

        # ---- 南侧道路路边平行车位（车道在南侧 -y）----
        if len(back_x) >= 2 and back_y is not None:
            xs = sorted(back_x)
            for k in range(len(xs) - 1):
                stalls.append(Stall(f"B{k}", "parallel", 0.5 * (xs[k] + xs[k + 1]),
                                    back_y, 0.0, xs[k + 1] - xs[k], back_w,
                                    (0.0, -1.0), "road_south"))

        # ---- 西/东侧街路边平行车位（车身沿 y，车道朝街心）----
        # 朝向统一取 -pi/2（车头朝南）：演示车辆从主路右转进入侧街是自北向南行驶，
        # 平行泊车的正确动作是「开过车位 → 停在车位南侧 → 倒车入位」，停好后车头
        # 自然朝南。若把东侧车位写成 +pi/2，车就得从南往北开过来才能入位，从主路
        # 出发要绕行整个街区；反过来也一样。所以两排都按「自北向南」标定。
        for side, pts in side_pts.items():
            if len(pts) < 2:
                continue
            xs = sorted(p[0] for p in pts)
            x = sum(xs) / len(xs)
            ys = sorted(p[1] for p in pts)
            lane_n = (1.0, 0.0) if side == "W" else (-1.0, 0.0)
            for k in range(len(ys) - 1):
                stalls.append(Stall(f"S{side}{k}", "parallel", x,
                                    0.5 * (ys[k] + ys[k + 1]), -math.pi / 2,
                                    ys[k + 1] - ys[k], side_w[side], lane_n,
                                    "road_west" if side == "W" else "road_east"))

        meta = {
            "frame": "world",
            "source_world": os.path.basename(world_path),
            "vehicle": {"length": LENGTH, "width": WIDTH,
                        "wheelbase": WHEELBASE, "max_steer": MAX_STEER,
                        "effective_max_steer": EFFECTIVE_MAX_STEER},
        }
        if ego_pose is not None:
            meta["nominal_spawn"] = {"x": ego_pose[0], "y": ego_pose[1],
                                     "yaw": ego_pose[2]}

        # 规划 ROI = 整个街区（含四条道路），但如果 world 里还有更远的东西就裁掉
        roi = (-100.0, 100.0, -45.0, 42.0)
        obstacles = [o for o in obstacles if _rect_in_box(o, roi, margin=2.0)]

        return cls(obstacles, stalls, meta, resolution, roi)

    # ---------------------------------------------------------------- 坐标系变换
    def to_frame(self, spawn):
        """world → map。spawn = (x, y, yaw)：自车生成位姿（world 帧）。"""
        c, s = math.cos(-spawn[2]), math.sin(-spawn[2])
        obs = []
        for r in self.obstacles:
            dx, dy = r.cx - spawn[0], r.cy - spawn[1]
            obs.append(Rect(c * dx - s * dy, s * dx + c * dy,
                            wrap(r.yaw - spawn[2]), r.hl, r.hw, r.name))
        stalls = [st.transformed(spawn) for st in self.stalls]
        meta = dict(self.meta)
        meta["frame"] = "map"
        meta["spawn"] = {"x": spawn[0], "y": spawn[1], "yaw": spawn[2]}
        # ROI 的四个角变换后取包围盒
        if self.roi:
            xmin, xmax, ymin, ymax = self.roi
            pts = [(xmin, ymin), (xmax, ymin), (xmax, ymax), (xmin, ymax)]
            mp = []
            for (px, py) in pts:
                dx, dy = px - spawn[0], py - spawn[1]
                mp.append((c * dx - s * dy, s * dx + c * dy))
            roi = (min(p[0] for p in mp), max(p[0] for p in mp),
                   min(p[1] for p in mp), max(p[1] for p in mp))
        else:
            roi = None
        load_staging_overrides()
        return Layout(obs, stalls, meta, self.resolution, roi)

    # ------------------------------------------------------------------ 存取
    def to_yaml_dict(self):
        return {"meta": self.meta, "resolution": self.resolution,
                "roi": list(self.roi) if self.roi else None,
                "obstacles": [r.to_dict() for r in self.obstacles],
                "stalls": [s.to_dict() for s in self.stalls]}

    @classmethod
    def from_yaml_dict(cls, d):
        obstacles = [Rect(o["cx"], o["cy"], o["yaw"], o["hl"], o["hw"],
                          o.get("name", "")) for o in d.get("obstacles", [])]
        stalls = [Stall.from_dict(s) for s in d.get("stalls", [])]
        roi = d.get("roi")
        return cls(obstacles, stalls, d.get("meta", {}),
                   d.get("resolution", 0.10), tuple(roi) if roi else None)

    def save(self, path):
        import yaml
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(self.to_yaml_dict(), f, allow_unicode=True,
                           sort_keys=False, default_flow_style=None)

    @classmethod
    def load(cls, path):
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            return cls.from_yaml_dict(yaml.safe_load(f))

    # ------------------------------------------------------------------ 栅格
    def bounds(self, pad=1.0):
        if self.roi:
            return self.roi
        xs, ys = [], []
        for r in self.obstacles:
            for (x, y) in r.corners:
                xs.append(x)
                ys.append(y)
        return (min(xs) - pad, max(xs) + pad, min(ys) - pad, max(ys) + pad)

    def occupancy_grid(self, resolution=None, roi=None):
        """生成占据栅格；roi 给定时只建该窗口（局部规划用）。"""
        from .grid import Grid
        res = float(resolution or self.resolution)
        xmin, xmax, ymin, ymax = roi if roi else self.bounds()
        grid = Grid.from_bounds(xmin, ymin, xmax, ymax, res)
        for r in self.obstacles:
            grid.fill_rect(r)
        return grid

    def find_stall(self, sid):
        for s in self.stalls:
            if s.id == sid:
                return s
        return None

    def stall_ids(self, region=None):
        return [s.id for s in self.stalls if region is None or s.region == region]
