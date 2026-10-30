#!/usr/bin/env python3
"""沿「出生点 → W4 预备位姿」的路线走 37.6 m，列出该处周围 5 m 内的障碍。

用途：真机日志里激光护栏反复命中 `lon≈3.6 lat≈+1.1`，需要确认那到底是什么。
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.geometry import body_gap                       # noqa: E402
from parking_planner.lot_layout import Layout                       # noqa: E402
from parking_planner.route_planner import (RouteGraph,              # noqa: E402
                                           default_graph_path)

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds', 'city_parking_lot.sdf')
SPAWN = (-45.0, 15.65, 0.0)


def main():
    arc = float(sys.argv[1]) if len(sys.argv) > 1 else 37.6
    lay = Layout.from_world(WORLD).to_frame(SPAWN)
    g = RouteGraph.load(default_graph_path()).to_frame(SPAWN)
    st = lay.find_stall('W4')
    route = g.plan((0.0, 0.0), st.staging_pose, start_heading=0.0)
    # 沿路线累加到 arc 米
    acc, p = 0.0, route[0]
    for a, b in zip(route, route[1:]):
        d = math.hypot(b[0] - a[0], b[1] - a[1])
        if acc + d >= arc:
            u = (arc - acc) / max(d, 1e-9)
            p = (a[0] + (b[0] - a[0]) * u, a[1] + (b[1] - a[1]) * u,
                 math.atan2(b[1] - a[1], b[0] - a[0]))
            break
        acc += d
        p = b
    print(f'路线全长 {sum(math.hypot(b[0]-a[0], b[1]-a[1]) for a, b in zip(route, route[1:])):.1f} m；'
          f'走 {arc} m 处 = ({p[0]:.2f},{p[1]:.2f},{math.degrees(p[2]):.1f}°)')
    print(f'该位姿处车体最小间隙 = {body_gap(p, lay.obstacles):+.3f} m')
    c, s = math.cos(p[2]), math.sin(p[2])
    rows = []
    for ob in lay.obstacles:
        d = math.hypot(ob.cx - p[0], ob.cy - p[1])
        if d > 8.0:
            continue
        lon = c * (ob.cx - p[0]) + s * (ob.cy - p[1])
        lat = -s * (ob.cx - p[0]) + c * (ob.cy - p[1])
        rows.append((d, lon, lat, ob.hl, ob.hw, ob.name))
    rows.sort()
    print(f'\n8 m 内障碍 {len(rows)} 个（lon 沿车头，lat 左正）：')
    for d, lon, lat, hl, hw, n in rows:
        # 障碍矩形到 车体坐标系 的最近横向距离
        cl_lon = abs(lon) - hl
        cl_lat = abs(lat) - hw
        print(f'  {n:16s} lon={lon:+6.2f} lat={lat:+6.2f} 半尺寸 {hl:.2f}x{hw:.2f} '
              f'（差 {max(cl_lon, cl_lat, 0.0):.2f} m 出界）')


if __name__ == '__main__':
    main()
