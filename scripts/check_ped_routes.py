#!/usr/bin/env python3
"""检查行人闭环路径是否「走在机动车道上」或「穿过车位」。

背景：`pedestrian_lot_inner` 原来的航点有三段正好压在干扰车的场内单向环上
（净距 0.00 m），于是车必然从人身上开过去 —— 实测行人被压到「离车体 0.0 m」。
这类问题靠肉眼看航点坐标是发现不了的，所以做成检查：

  1. 行人路径与任何**车位矩形**的最小距离必须 > 1.5 m（不能穿车位；
     车位里停着车，车让不了）；
  2. 与**场内机动车道**的并行段净距必须 > 2.5 m（小于这个值就是人车同道，
     车会不停让行甚至压上去）；**交叉**允许 —— 那是过街，车会让人。
  3. 与每个车位的**入位走廊**（预备位姿 → 车位目标的直线倒车段，沿线上
     铺满自车车身矩形）净距必须 > 2.0 m。

第 3 条是补的：之前只量「离车位目标点多远」，看不见**预备位姿**。
`pedestrian_lot_inner` 走在 x=±11.5 上时就报 OK（离 W3 目标点 2.60 m），
可自车必须在 x=-12.5 停住、再往回退 4.2 m 进车位 —— 行人那条线离倒车
走廊只有 1.0 m，实测就是「人站在车尾要退的线上」，泊车卡死。

航点直接从 `parking_demo.launch.py` 的 `PED_ROUTES` 里解析，避免两处漂移。

用法：`python3 scripts/check_ped_routes.py`
"""
import ast
import math
import os
import re
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from parking_planner.lot_layout import Layout                     # noqa: E402

WORLD = os.path.join(WS, 'src', 'parking_lot_city', 'worlds',
                     'city_parking_lot.sdf')
LAUNCH = os.path.join(WS, 'src', 'parking_planner', 'launch',
                      'parking_demo.launch.py')

STALL_MIN = 1.5      # 行人离车位矩形的最小距离
LANE_MIN = 2.5       # 与机动车道的并行段最小净距
CORR_MIN = 2.0       # 离车位入位走廊（含自车车身）的净距

EGO_HL = 2.35        # 自车半长
EGO_HW = 0.925       # 自车半宽


def load_routes():
    text = open(LAUNCH, encoding='utf-8').read()
    m = re.search(r'PED_ROUTES = \((.*?)\n\s*\)', text, re.S)
    if not m:
        raise SystemExit('没在 launch 里找到 PED_ROUTES')
    # 只取引号里的内容再拼起来：注释、换行、缩进都不影响
    joined = ''.join(re.findall(r'"([^"]*)"', m.group(1)))
    routes = {}
    for chunk in joined.split('|'):
        if '=' not in chunk:
            continue
        name, pts = chunk.split('=', 1)
        xy = []
        for seg in pts.split(';'):
            if ',' in seg:
                a, b = seg.split(',')
                xy.append((float(a), float(b)))
        routes[name.strip()] = xy
    return routes


def load_traffic():
    """场内机动车道：直接从 traffic_driver.py 的 ROUTE 里取。"""
    p = os.path.join(WS, 'src', 'scene_agents', 'scene_agents',
                     'traffic_driver.py')
    text = open(p, encoding='utf-8').read()
    body = text.split('ROUTE = [', 1)[1].split(']', 1)[0]
    return [(float(a), float(b)) for a, b in
            re.findall(r'\(\s*(-?\d+\.?\d*)\s*,\s*(-?\d+\.?\d*)\s*\)', body)]


def stall_poly(st):
    c, s = math.cos(st.yaw), math.sin(st.yaw)
    hl, hw = st.l * 0.5, st.w * 0.5
    return [(st.cx + c * u - s * v, st.cy + s * u + c * v)
            for (u, v) in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def pt_seg(p, a, b):
    dx, dy = b[0] - a[0], b[1] - a[1]
    L2 = dx * dx + dy * dy
    if L2 < 1e-12:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    t = max(0.0, min(1.0, ((p[0] - a[0]) * dx + (p[1] - a[1]) * dy) / L2))
    return math.hypot(p[0] - a[0] - dx * t, p[1] - a[1] - dy * t)


def cross(a, b, c, d):
    def ccw(p, q, r):
        return (r[1] - p[1]) * (q[0] - p[0]) - (q[1] - p[1]) * (r[0] - p[0])
    return ccw(c, d, a) * ccw(c, d, b) < 0 and ccw(a, b, c) * ccw(a, b, d) < 0


def seg_dist(a, b, poly, closed):
    m = len(poly)
    rng = range(m) if closed else range(m - 1)
    for j in rng:
        if cross(a, b, poly[j], poly[(j + 1) % m]):
            return -1.0
    best = float('inf')
    for j in rng:
        best = min(best, pt_seg(a, poly[j], poly[(j + 1) % m]),
                   pt_seg(b, poly[j], poly[(j + 1) % m]))
    return best


def ego_poly(x, y, yaw, hl=EGO_HL, hw=EGO_HW):
    c, s = math.cos(yaw), math.sin(yaw)
    return [(x + c * u - s * v, y + s * u + c * v)
            for (u, v) in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw))]


def corridor_polys(st, step=0.5):
    """入位走廊：预备位姿 → 车位目标，沿直线铺满自车车身矩形。

    ackermann 倒车段的航向不变，所以沿线上每个点铺一个车身矩形就够了；
    平行车位还有一段横向偏移，因此按两点连线插值而不是只沿车位轴。
    """
    gx, gy, gyaw = st.goal_pose
    sx, sy, _ = st.staging_pose
    L = math.hypot(sx - gx, sy - gy)
    n = max(2, int(L / step) + 1)
    out = []
    for i in range(n):
        t = i / (n - 1.0)
        out.append(ego_poly(gx + (sx - gx) * t, gy + (sy - gy) * t, gyaw))
    return out


def main():
    layout = Layout.from_world(WORLD)
    stalls = [(st.id, stall_poly(st)) for st in layout.stalls]
    corridors = [(st.id, corridor_polys(st)) for st in layout.stalls]
    lanes = load_traffic()
    bad = 0
    for name, pts in load_routes().items():
        n = len(pts)
        dmin, sid0 = 9e9, ''
        stall_at = None
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            for (sid, poly) in stalls:
                d = seg_dist(a, b, poly, True)
                if d < dmin:
                    dmin, sid0, stall_at = d, sid, (i, a, b)
        lane_bad, crossings, lane_at = [], 0, None
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            d = seg_dist(a, b, lanes, False)
            if d < 0:
                crossings += 1
            elif d < LANE_MIN:
                lane_bad.append(d)
                if lane_at is None:
                    lane_at = (i, a, b)
        # 入位走廊：取整条路径上离任一走廊最近的那个点到走廊的距离
        # 与车道同理 —— **横穿**允许（行人过街/穿场），只有**并行贴着**才算错。
        # 否则东侧那条线无解：SE 排 6 个车位的倒车走廊连成 x∈[28,32.6]、
        # y∈[-28.5,13] 的一整条带子，南北向的走廊把东侧完全隔开，
        # 从 x=21.6 走到 x=36.2 不可能不穿。
        cmin, cid, c_cross = 9e9, '', 0
        for i in range(n):
            a, b = pts[i], pts[(i + 1) % n]
            for (sid, polys) in corridors:
                for poly in polys:
                    d = seg_dist(a, b, poly, True)
                    if d < 0:
                        c_cross += 1
                        break
                    if d < cmin:
                        cmin, cid = d, sid
        ok_stall = dmin > STALL_MIN
        ok_lane = not lane_bad
        ok_corr = cmin > CORR_MIN
        flag = 'OK ' if (ok_stall and ok_lane and ok_corr) else 'FAIL'
        if flag == 'FAIL':
            bad += 1
        print(f'{flag} {name:22s} 离车位 {dmin:5.2f} m（{sid0}）'
              f'{"  ← 太近" if not ok_stall else ""} | '
              f'离入位走廊 {cmin:5.2f} m（{cid}）'
              f'{"  ← 贴着倒车走廊" if not ok_corr else ""}'
              f'（穿 {c_cross} 处） | '
              f'并行段离车道 {min(lane_bad) if lane_bad else 9.9:5.2f} m'
              f'{"  ← 人车同道" if not ok_lane else ""} | 过街 {crossings} 处')
        if lane_at is not None and os.environ.get('VERBOSE'):
            i, a, b = lane_at
            print(f'      ↑ 最近车道段 [{i}] {a} → {b}')
        if stall_at is not None and os.environ.get('VERBOSE') and not ok_stall:
            i, a, b = stall_at
            print(f'      ↑ 最近车位段 [{i}] {a} → {b}（{sid0}）')
    print(f'\n通过 {len(load_routes()) - bad}/{len(load_routes())}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
