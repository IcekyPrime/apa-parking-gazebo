#!/usr/bin/env python3
"""干扰车让行判据的单测（纯几何，不起仿真）。

背景：真机截图里蓝色干扰车撞上了正在过道里横着停车的自车。旧的 `_blocked`
有三条互相叠加的漏洞：
  1. `dist > 5.0 且 |Δyaw| > 60° → continue`（当成另一条车道，直接不看）；
  2. 硬性间距 `hard_radius = 3.0 m` 是**中心距**，比两车半长之和 4.35 m 还小，
     永远不会先于物理接触触发；
  3. 横向门限用 `corridor_half` 判 центр点，斜着摆的车会被漏掉。
现在改成「前方走廊矩形 ∩ 对方车身矩形」（SAT）。

用法：`python3 scripts/selftest_traffic_yield.py`
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'scene_agents'))

from scene_agents.path_agent import CAR_HL, CAR_HW, rects_overlap  # noqa: E402


class Fake:
    """只带 `_blocked` 需要的字段，避免起 rclpy。"""

    def __init__(self):
        self.stop_r = 9.0
        self.cor_half = 2.4
        self.cor_back = 0.0
        self.hard_r = 4.0
        self.ped_hard = 3.0
        self.ped_stop = 7.0
        self.peds = []
        self.others = {}

    _blocked = None            # 由下面手动绑定


def blocked(self, x, y, th, others, peds=()):
    """与 path_agent.PathAgent._blocked 同构的独立实现（判据一致）。"""
    c, s = math.cos(th), math.sin(th)
    lc = 0.5 * (self.cor_back + self.stop_r)
    chl = 0.5 * (self.stop_r - self.cor_back)
    cx, cy = x + c * lc, y + s * lc
    for (ox, oy, oyaw) in others:
        if math.hypot(ox - x, oy - y) > self.stop_r + 4.0:
            continue
        if rects_overlap(cx, cy, th, chl, self.cor_half,
                         ox, oy, oyaw, CAR_HL, CAR_HW):
            return True
        if (math.hypot(ox - x, oy - y) < self.hard_r
                and rects_overlap(x, y, th, CAR_HL, CAR_HW,
                                  ox, oy, oyaw, CAR_HL, CAR_HW)):
            return True
    for (px, py) in peds:
        dx, dy = px - x, py - y
        if math.hypot(dx, dy) < self.ped_hard:
            return True
        lon = c * dx + s * dy
        lat = -s * dx + c * dy
        if -1.5 < lon < self.ped_stop and abs(lat) < self.cor_half:
            return True
    return False


D = math.radians

# (说明, 本车位姿, 对方位姿, 期望)
CASES = [
    ('正前 4 m 同向车',        (0, 0, 0),   [(4.0, 0.0, 0.0)],        True),
    ('正前 8 m 同向车',        (0, 0, 0),   [(8.0, 0.0, 0.0)],        True),
    ('正前 14 m 同向车',       (0, 0, 0),   [(14.0, 0.0, 0.0)],       False),
    # 核心用例：本车沿车道直行，"其他车"是横着停在过道里的自车（真机截图那一幕）
    ('横停自车 6 m 前',        (0, 0, 0),   [(6.0, 0.0, D(90))],      True),
    ('横停自车 8.5 m 前',      (0, 0, 0),   [(8.5, 0.0, D(90))],      True),
    ('横停自车 12 m 前',       (0, 0, 0),   [(12.0, 0.0, D(90))],     False),
    # 对向/邻近车道不该误触发（旧判据靠航向豁免，新判据靠几何）
    ('邻道并行 6 m 横向',      (0, 0, 0),   [(0.0, 6.0, 0.0)],        False),
    ('邻道对向 6 m 横向',      (0, 0, 0),   [(5.0, 6.0, D(180))],     False),
    ('正后方 5 m 的车',        (0, 0, 0),   [(-5.0, 0.0, 0.0)],       False),
    ('正后方 8 m 的车',        (0, 0, 0),   [(-8.0, 0.0, 0.0)],       False),
    # 侧面被贴上（别人撞我）→ 兜底停车
    ('侧向已重叠',             (0, 0, 0),   [(0.0, 1.6, D(90))],      True),
    # 斜前方 60°、5 m —— 旧判据会豁免掉
    ('斜前方 60°/5 m',         (0, 0, 0),   [(2.5, 4.33, D(60))],     True),
]

# 行人
PED_CASES = [
    ('正前 5 m 行人',   (0, 0, 0), [(5.0, 0.0)],   True),
    ('正前 9 m 行人',   (0, 0, 0), [(9.0, 0.0)],   False),
    ('侧后 135°/2 m 行人', (0, 0, 0), [(-1.41, 1.41)], True),
]


def main():
    f = Fake()
    bad = 0
    print(f'车辆外廓 {CAR_HL * 2:.2f} × {CAR_HW * 2:.2f} m，'
          f'走廊 lon∈[{f.cor_back:.1f}, {f.stop_r:.1f}]、lat∈±{f.cor_half:.1f}，'
          f'车身重叠兜底 {f.hard_r:.1f} m')
    for name, me, others, want in CASES:
        got = blocked(f, me[0], me[1], me[2], others)
        ok = (got == want)
        bad += 0 if ok else 1
        print(f'  [{"OK" if ok else "NG"}] {name:22s} '
              f'{"停车让行" if got else "继续行驶"}   期望 '
              f'{"停车让行" if want else "继续行驶"}')
    for name, me, peds, want in PED_CASES:
        got = blocked(f, me[0], me[1], me[2], [], peds)
        ok = (got == want)
        bad += 0 if ok else 1
        print(f'  [{"OK" if ok else "NG"}] {name:22s} '
              f'{"停车让行" if got else "继续行驶"}   期望 '
              f'{"停车让行" if want else "继续行驶"}')
    print(f'\n{"全部通过" if bad == 0 else f"失败 {bad} 条"}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
