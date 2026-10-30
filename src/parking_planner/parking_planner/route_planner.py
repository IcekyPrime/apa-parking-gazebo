"""全局路径规划：车道图 + 带转向代价的 Dijkstra + 圆弧圆滑。

用途：自车可以在主路**任意位置**生成，而泊车机动的起点是目标车位的「预备位姿」。
两者之间可能隔着几十米、要过路口，所以先在图（车道中心线）上搜一条路线，圆滑后
交给跟踪器驾驶；到了目标车位所在车道后，最后几米再交给 Hybrid A* 做精细泊车。

三级分工的理由
--------------
整街区 200 m 见方，若直接用 0.25 m 的 Hybrid A* 栅格，单是 2D 启发式的 Dijkstra
就要跑 80 万格，搜索也不可接受。「图搜索 + 局部运动规划」是自动驾驶里的常规做法。

两条必须守住的交通规则
--------------------
1) **单向车道**：主路东行/西行各占一侧，侧街为单向（自北向南）。图里的车道链是
   有向的，否则 Dijkstra 会让车逆向行驶抄近路。
2) **禁止掉头**：转向代价里对「接近 180° 的折返」施加巨大惩罚。车道链与侧街共用
   交叉口节点，如果没有这个惩罚，从西行车道去停车场（在南侧）会被规划成
   「左转进侧街 → 开 16 m → 再折回东行车道」，而那是个半径 0.8 m 的掉头，车根本
   做不出来。加了惩罚后会自动改走「侧街 → 南侧道路 → 东侧街 → 东行车道 → 停车场」
   这个合法绕行。
"""

import heapq
import math
import os

from .geometry import wrap
from .lot_layout import MIN_TURN_RADIUS

# 掉头惩罚（折算成米）。一个 180° 折返相当于多绕 800 m，
# 而一个 90° 转弯的惩罚为 0，所以正常路口转弯不受影响。
TURN_PENALTY = 800.0
# 起点节点在车头反方向的惩罚（折算成米）。自车正东行驶、西侧街路口刚好在身后
# 9 m 时，不加这个惩罚会规划成「先向西开回路口再进街」，等于倒车。
START_BACK_PENALTY = 300.0


class RouteGraph:
    def __init__(self, nodes, edges, chains=(), merge_tol=1.0):
        # 路口处同一位置可能被标成两个节点。不合并的话 Dijkstra 会绕到相邻节点
        # 再折返，在路线上造出死胡同掉头。
        raw = {k: (float(v[0]), float(v[1])) for k, v in nodes.items()}
        alias, kept = {}, {}
        for k, xy in raw.items():
            rep = None
            for k2, xy2 in kept.items():
                if math.hypot(xy[0] - xy2[0], xy[1] - xy2[1]) <= merge_tol:
                    rep = k2
                    break
            if rep is None:
                kept[k] = xy
                alias[k] = k
            else:
                alias[k] = rep
        self.nodes = kept
        self.adj = {k: [] for k in kept}       # 有向：只存出边
        self.arcs = []                          # (a, b, 双向?) 供 to_frame 复用
        seen = set()

        def add(a, b, two_way):
            a2, b2 = alias.get(a), alias.get(b)
            if a2 is None or b2 is None or a2 == b2:
                return
            key = (a2, b2) if two_way else ("1", a2, b2)
            if key in seen:
                return
            seen.add(key)
            d = math.hypot(self.nodes[a2][0] - self.nodes[b2][0],
                           self.nodes[a2][1] - self.nodes[b2][1])
            self.adj[a2].append((b2, d))
            if two_way:
                self.adj[b2].append((a2, d))
            self.arcs.append((a2, b2, two_way))

        for e in edges:
            add(e[0], e[1], True)
        for chain in chains:
            for a, b in zip(chain, chain[1:]):
                add(a, b, False)

    # ------------------------------------------------------------------ 构造
    @classmethod
    def load(cls, path):
        import yaml
        with open(path, "r", encoding="utf-8") as f:
            d = yaml.safe_load(f)
        return cls(d["nodes"], d.get("edges") or [], d.get("chains") or [])

    def to_frame(self, spawn):
        """world → map 变换（自车生成位姿为 map 帧原点）。"""
        c, s = math.cos(-spawn[2]), math.sin(-spawn[2])
        nodes = {}
        for k, (x, y) in self.nodes.items():
            dx, dy = x - spawn[0], y - spawn[1]
            nodes[k] = (c * dx - s * dy, s * dx + c * dy)
        edges = [[a, b] for a, b, two in self.arcs if two]
        chains = []
        # 有向弧重新拼回链，交给构造函数按方向建边
        for a, b, two in self.arcs:
            if not two:
                chains.append([a, b])
        return RouteGraph(nodes, edges, chains)

    @property
    def edges(self):
        return [[a, b] for a, b, _two in self.arcs]

    # ------------------------------------------------------------------ 搜索
    def nearest(self, xy, ids=None):
        best, bd = None, float("inf")
        for k, (x, y) in self.nodes.items():
            if ids is not None and k not in ids:
                continue
            d = math.hypot(x - xy[0], y - xy[1])
            if d < bd:
                bd, best = d, k
        return best, bd

    def _reverse_dist_to(self, goal_id):
        """从 goal 在反向图上跑一次 Dijkstra，得到「各节点到 goal 的最短路长」。

        用来挑选起点节点：既不能离自车太远，也要在行进方向上（否则会出现
        「先倒回上一个节点再出发」的折返）。
        """
        radj = {k: [] for k in self.nodes}
        for a, lst in self.adj.items():
            for b, w in lst:
                radj[b].append((a, w))
        dist = {goal_id: 0.0}
        pq = [(0.0, goal_id)]
        while pq:
            d, u = heapq.heappop(pq)
            if d > dist.get(u, float("inf")):
                continue
            for v, w in radj[u]:
                nd = d + w
                if nd < dist.get(v, float("inf")) - 1e-9:
                    dist[v] = nd
                    heapq.heappush(pq, (nd, v))
        return dist

    def _turn_cost(self, p, u, v, penalty):
        if p is None:
            return 0.0
        pu, cu, cv = self.nodes[p], self.nodes[u], self.nodes[v]
        ax, ay = cu[0] - pu[0], cu[1] - pu[1]
        bx, by = cv[0] - cu[0], cv[1] - cu[1]
        na, nb = math.hypot(ax, ay), math.hypot(bx, by)
        if na < 1e-9 or nb < 1e-9:
            return 0.0
        cosang = (ax * bx + ay * by) / (na * nb)
        # 只有「折返」才罚。注意阈值**不能**取 90°：
        # 主路东行车道转入东侧街时（侧街中心线向路中偏 0.6 m 后）夹角是 94.5°，
        # 按 90° 判会被当成折返罚 62 分，Dijkstra 于是绕行「西街 + 南侧道路」
        # （119.5 m 对 95.7 m）—— 真机 SE1 三连超时的直接原因
        # （日志 `生成全局路线：116.1 m`，末位置在西街 (-28.6, 1.1)）。
        # 放宽到 120°：正常路口转弯（含轻微偏斜）免费，真正的掉头折返
        # （>120°）仍按 penalty 罚；路线圆滑 `_round_corners` 最多支持 150° 转角，
        # 120° 内的转角对车辆仍是可跟随的。
        if cosang >= -0.5:          # ≤120°
            return 0.0
        return penalty * max(0.0, -cosang)

    def shortest_path(self, start_id, goal_id, turn_penalty=TURN_PENALTY):
        """带转向代价的 Dijkstra；状态是 (上一个节点, 当前节点)。"""
        start = (None, start_id)
        dist = {start: 0.0}
        parent = {}
        pq = [(0.0, start_id, None)]
        while pq:
            d, u, p = heapq.heappop(pq)
            st = (p, u)
            if d > dist.get(st, float("inf")) + 1e-9:
                continue
            if u == goal_id:
                out = [u]
                cur = st
                while parent.get(cur) is not None:
                    pp, pu = parent[cur]
                    out.append(pu)
                    cur = (pp, pu)
                return list(reversed(out))
            for v, w in self.adj[u]:
                if v == p:
                    continue            # 原路返回没有意义
                nst = (u, v)
                nd = d + w + self._turn_cost(p, u, v, turn_penalty)
                if nd < dist.get(nst, float("inf")) - 1e-9:
                    dist[nst] = nd
                    parent[nst] = st
                    heapq.heappush(pq, (nd, v, u))
        return None

    # ------------------------------------------------------------------ 对外
    def plan(self, start_xy, goal_pose, start_heading=None, snap_radius=60.0,
             append_min=12.0, start_radius=12.0):
        """返回 [(x, y, yaw), ...] 的平滑折线；无法连通返回 None。

        append_min：最近节点与目标的距离小于该值时，只有「节点→目标」的方向与
        最后一段行进方向大致一致（夹角 < 55°）才把目标接进路线；正交的情况
        （场内垂直车位）留给 Hybrid A*，否则末端会多出一个跟不住的 90° 急弯。

        start_radius：起点节点的搜索半径。**必须有这个上限** —— 起点节点的打分
        是 `离自车距离 + 到目标的图距离`，如果不设上限，靠近目标的车道节点会「飞」
        过来接自车（比如自车在主路上、起点节点却被选成 28 m 外西侧街里的节点），
        于是「自车 → 首个节点」变成一条横穿人行道的斜线，而曲率检查又看不出问题。
        主路节点按 9~10 m 一档布设，12 m 半径下必定能命中。
        """
        g_id, g_d = self.nearest((goal_pose[0], goal_pose[1]))
        if g_d > snap_radius:
            return None
        gdist = self._reverse_dist_to(g_id)
        # 起点节点：在「离自车近」「在行进方向上」「到目标的图距离短」三者间折中
        hx = math.cos(start_heading) if start_heading is not None else None
        hy = math.sin(start_heading) if start_heading is not None else None
        s_id, s_best, s_near = None, float("inf"), float("inf")
        for k, xy in self.nodes.items():
            if k not in gdist:
                continue
            dx, dy = xy[0] - start_xy[0], xy[1] - start_xy[1]
            near = math.hypot(dx, dy)
            if near > start_radius:
                continue
            if k == g_id and near > 6.0:
                continue          # 目标节点本身不能当起点（会退化成直线）
            tot = near + gdist[k]
            if hx is not None and near > 1.0:
                # 首段方向必须与车头一致：否则会出现「先往回开 9 m 再走」的倒车，
                # 而那在真实车道上是不可能的（该绕行就得绕行）。
                tot += START_BACK_PENALTY * max(0.0, -(hx * dx + hy * dy) / near)
            if tot < s_best:
                s_best, s_id, s_near = tot, k, near
        if s_id is None:
            return None
        ids = self.shortest_path(s_id, g_id)
        if ids is None:
            return None

        pts = [self.nodes[i] for i in ids]
        if s_near > 1.5:      # 自车还没到第一个节点，接在路线头上
            pts.insert(0, (start_xy[0], start_xy[1]))

        if g_d < 1.6:
            # 最近节点本来就落在预备位姿上（路侧车位节点就是按预备位姿标定的），
            # 再补一段 0.2 m 的碎线只会让离散化后的曲率估计爆掉，没有意义。
            pass
        elif g_d > append_min:
            pts.append((goal_pose[0], goal_pose[1]))
        else:
            gx, gy = goal_pose[0] - pts[-1][0], goal_pose[1] - pts[-1][1]
            gd = math.hypot(gx, gy)
            if gd < 1e-6 or len(pts) < 2:
                pts.append((goal_pose[0], goal_pose[1]))
            else:
                ex, ey = pts[-1][0] - pts[-2][0], pts[-1][1] - pts[-2][1]
                el = math.hypot(ex, ey)
                cosang = 1.0 if el < 1e-6 else (ex * gx + ey * gy) / (el * gd)
                if cosang > math.cos(math.radians(55.0)):
                    pts.append((goal_pose[0], goal_pose[1]))

        poly = _round_corners(pts, radius=MIN_TURN_RADIUS + 0.3, samples=10)
        return _densify(poly, step=0.3, final_yaw=goal_pose[2],
                        start_heading=start_heading)


# ---------------------------------------------------------------------- 工具
def _unit(a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    d = math.hypot(dx, dy)
    if d < 1e-9:
        return (0.0, 0.0), 0.0
    return (dx / d, dy / d), d


def _round_corners(pts, radius=MIN_TURN_RADIUS, samples=10):
    """把折线转角替换成**真正的定半径圆弧**。

    以前用二次 Bezier 近似，它在中点的最小曲率半径只有切线长的 1/√2 倍；而切线
    长又被夹在相邻边长的 45% 以内。于是停车场内 8 m 见方的过道里实际生成的半径
    只有 2 m 出头，远超车辆 5.74 m 的最小转弯半径 —— 跟踪时横向偏差一路发散到
    8 m，车直接开出场外。

    这里改成显式圆弧：90° 转角的切线长 T = R·tan(φ/2) = R，比 Bezier 省 30% 的
    纵向空间；再按边长成比例缩放，保证生成的曲率不超过车辆能力。
    """
    n = len(pts)
    if n < 3:
        return list(pts)

    dirs, lens = [], []
    for i in range(n - 1):
        u, d = _unit(pts[i], pts[i + 1])
        dirs.append(u)
        lens.append(d)

    phi = [0.0] * n
    need = [0.0] * n
    for i in range(1, n - 1):
        cross = dirs[i - 1][0] * dirs[i][1] - dirs[i - 1][1] * dirs[i][0]
        dot = dirs[i - 1][0] * dirs[i][0] + dirs[i - 1][1] * dirs[i][1]
        phi[i] = abs(math.atan2(cross, dot))
        if math.radians(2.0) < phi[i] < math.radians(160.0):
            need[i] = radius * math.tan(min(phi[i], math.radians(150.0)) * 0.5)
        # 接近 180° 的是折返（图上的路线错了），不圆滑，交给跟踪器换挡

    scale = [1.0] * (n - 1)
    for i in range(n - 1):
        tot = need[i] + need[i + 1]
        if tot > 1e-9 and tot > 0.95 * lens[i]:
            scale[i] = 0.95 * lens[i] / tot

    out = [pts[0]]
    for i in range(1, n - 1):
        if need[i] < 1e-6:
            out.append(pts[i])
            continue
        t = min(need[i] * scale[i - 1], need[i] * scale[i])
        half = min(phi[i], math.radians(150.0)) * 0.5
        if t < 0.15:
            out.append(pts[i])
            continue
        r = t / math.tan(half)          # 受可用切线长限制后的实际半径
        u1, u2 = dirs[i - 1], dirs[i]
        a = (pts[i][0] - u1[0] * t, pts[i][1] - u1[1] * t)
        b = (pts[i][0] + u2[0] * t, pts[i][1] + u2[1] * t)
        out.append(a)
        sign = 1.0 if (u1[0] * u2[1] - u1[1] * u2[0]) > 0.0 else -1.0
        cx = a[0] - u1[1] * sign * r
        cy = a[1] + u1[0] * sign * r
        a0 = math.atan2(a[1] - cy, a[0] - cx)
        dth = wrap(math.atan2(b[1] - cy, b[0] - cx) - a0)
        for k in range(1, samples):
            th = a0 + dth * (k / samples)
            out.append((cx + r * math.cos(th), cy + r * math.sin(th)))
        out.append(b)
    out.append(pts[-1])
    return out


def _densify(pts, step=0.3, final_yaw=None, start_heading=None):
    """按步长插值并计算每点的朝向；最后一个点的朝向强制为泊车预备位姿朝向。"""
    if len(pts) < 2:
        return []
    dense = []
    for i in range(len(pts) - 1):
        x0, y0 = pts[i]
        x1, y1 = pts[i + 1]
        seg = math.hypot(x1 - x0, y1 - y0)
        n = max(1, int(seg / step))
        for k in range(n):
            u = k / n
            dense.append((x0 + (x1 - x0) * u, y0 + (y1 - y0) * u))
    dense.append(pts[-1])

    out = []
    for i, (x, y) in enumerate(dense):
        j = min(i + 1, len(dense) - 1)
        k = max(0, j - 1)
        dx = dense[j][0] - dense[k][0]
        dy = dense[j][1] - dense[k][1]
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            yaw = out[-1][2] if out else (start_heading or 0.0)
        else:
            yaw = math.atan2(dy, dx)
        out.append((x, y, yaw))
    if out and final_yaw is not None:
        n = min(len(out), 12)
        for i in range(n):
            idx = len(out) - n + i
            w = (i + 1) / n
            out[idx] = (out[idx][0], out[idx][1],
                        wrap(out[idx][2] + wrap(final_yaw - out[idx][2]) * w))
    return out


def default_graph_path(package_share=None):
    if package_share:
        p = os.path.join(package_share, "config", "route_graph.yaml")
        if os.path.exists(p):
            return p
    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    return os.path.join(here, "config", "route_graph.yaml")
