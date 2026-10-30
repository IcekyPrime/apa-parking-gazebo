#!/usr/bin/env python3
"""离线检查：从主路多个位置到各类车位，全局路线的曲率是否超出车辆能力。"""
import math
import os
import sys

sys.path.insert(0, os.path.expanduser("~/apa_ws/src/parking_planner"))

from parking_planner.lot_layout import Layout, MIN_TURN_RADIUS  # noqa: E402
from parking_planner.route_planner import RouteGraph, default_graph_path  # noqa: E402

WORLD = os.path.expanduser("~/apa_ws/src/parking_lot_city/worlds/city_parking_lot.sdf")
KMAX = 1.0 / MIN_TURN_RADIUS + 1e-3


def inside(rect, x, y, margin=0.5):
    """点是否落在矩形（含 margin）内。"""
    c, s = math.cos(-rect.yaw), math.sin(-rect.yaw)
    dx, dy = x - rect.cx, y - rect.cy
    u, v = c * dx - s * dy, s * dx + c * dy
    return abs(u) <= rect.hl - margin and abs(v) <= rect.hw - margin


VEHICLE_NAMES = ("parked_car", "parked_", "city_car", "traffic", "ego_car")


def wall_hits(layout, path):
    """统计路线穿过「静态基础设施」（建筑、绿化岛、立柱、树、人行道）的点数。

    车辆类障碍（停着的车、干扰车）不算 —— 车道中心线本来就画在它们的位置上，
    动态/停放车辆由局部规划与风险评估处理，全局路线只负责沿车道走。
    """
    infra = [o for o in layout.obstacles
             if not any(s in o.name for s in VEHICLE_NAMES)]
    n = 0
    for p in path:
        for o in infra:
            if inside(o, p[0], p[1]):
                n += 1
                break
    return n


def max_curv(path, window=2.0):
    """用 2 m 弧长窗口上的航向变化率估计曲率。

    不能直接用相邻三点：路线是按 0.3 m 步长离散的，一段 R=5.4 m 的圆弧上相邻
    两点的转角有 9°，三点公式会把曲率放大一倍（实测量出 R=2.7 m 的假象）。
    取 2 m 窗口后，真圆弧给出 Δθ=2/R，而真正的急弯（90° 挤在 2 m 内）仍会被
    抓出来，判据依然有效。
    """
    n = len(path)
    worst, where = 0.0, None
    s = [0.0]
    for i in range(1, n):
        s.append(s[-1] + math.hypot(path[i][0] - path[i - 1][0],
                                    path[i][1] - path[i - 1][1]))
    j = 0
    for i in range(1, n - 1):
        while j < n - 1 and s[j] - s[i] < window / 2.0:
            j += 1
        k = i
        while k > 0 and s[i] - s[k] < window / 2.0:
            k -= 1
        a, b, c = path[k], path[i], path[j]
        h1 = math.atan2(b[1] - a[1], b[0] - a[0])
        h2 = math.atan2(c[1] - b[1], c[0] - b[0])
        ds = s[j] - s[k]
        if ds < 1e-6:
            continue
        dth = abs(math.atan2(math.sin(h2 - h1), math.cos(h2 - h1)))
        kk = dth / ds
        if kk > worst:
            worst, where = kk, (round(b[0], 1), round(b[1], 1))
    return worst, where


def main():
    raw = Layout.from_world(WORLD)
    graph_world = RouteGraph.load(default_graph_path())
    spawns = [(-45.0, 15.65, 0.0), (-80.0, 15.65, 0.0), (-20.0, 15.65, 0.0),
              (0.0, 24.35, math.pi), (60.0, 24.35, math.pi), (-50.0, 24.35, math.pi),
              (-88.0, 15.65, 0.0), (88.0, 24.35, math.pi)]
    targets = ["W0", "W5", "W9", "E0", "E8", "P0", "P1", "P3",
               "SW0", "SW1", "SW2", "SE1", "SE4", "B0", "B3"]
    ids = raw.stall_ids()
    print(f"车位总数 {len(ids)}：{ids}")
    bad = 0
    for sp in spawns:
        layout = raw.to_frame(sp)
        graph = graph_world.to_frame(sp)
        for tid in targets:
            st = layout.find_stall(tid)
            if st is None or tid not in ids:
                continue
            staging = st.staging_pose
            # 布局与车道图都已变换到 map 帧，自车生成位姿即 map 原点
            route = graph.plan((0.0, 0.0), staging, start_heading=0.0)
            if not route:
                print(f"  !! {sp[:2]} -> {tid}: 无路线")
                bad += 1
                continue
            length = sum(math.hypot(b[0] - a[0], b[1] - a[1])
                         for a, b in zip(route, route[1:]))
            straight = math.hypot(staging[0], staging[1])
            k, where = max_curv(route)
            r = 1.0 / k if k > 1e-9 else float("inf")
            hits = wall_hits(layout, route)
            ok = k <= KMAX and hits == 0
            if not ok:
                bad += 1
            print(f"  {'OK ' if ok else 'BAD'} {sp[:2]} yaw={sp[2]:+.2f} -> {tid:4s} "
                  f"({st.kind:13s}/{st.region:9s}) 长度 {length:6.1f} m "
                  f"(直线 {straight:5.1f}) 最大曲率 {k:.4f} (R={r:5.2f}) @ {where} "
                  f"穿墙点 {hits}")
    print(f"\n超过车辆能力(R<{MIN_TURN_RADIUS:.2f} m)的路线数：{bad}")


if __name__ == "__main__":
    main()
