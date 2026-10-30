#!/usr/bin/env python3
"""行人 / 自车「前方扇形」让行判据的纯几何单测（不启 Gazebo）。

两侧各有一个扇形判据，方向相反：

* `pedestrian_agent._sector_dist`：行人以**自己的行进方向**为轴取 ±fov/2，用射线
  与车体矩形求交，返回扇形内最近距离。它决定行人「什么时候该停」。
* `planner_node.ParkingPlanner._ped_in_front`：自车以**自己的行驶方向**为轴
  （倒车时转 180°）取 ±ped_fov/2，只对扇形内、离车体矩形小于 ped_yield 的行人
  停车。它决定自车「什么时候该等」。

两者都从「无方向的半径」改成扇形，修的同一个问题：行人明明已经从车侧/车后
走开，却还在互相等 —— 旧的圆形判据会让人白站、让车白白停车，进而触发
`_no_progress` 重规划，车向外挪又撞上人。

注意车体是 4.4 m × 2.0 m 的大矩形：一台停在正侧方 3 m 的车，它的前后角仍
伸进行人的 ±70° 扇形里，所以「侧方」不一定等于 inf。真正的判据是**距离**与
阈值的比较，测试按这个语义断言。

用法：`python3 scripts/selftest_ped_sector.py`
"""
import math
import os
import sys
import types

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'scene_agents'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from scene_agents.pedestrian_agent import PedestrianAgent   # noqa: E402
from parking_planner.planner_node import ParkingPlanner     # noqa: E402

HALF_L = 2.20            # 与 body_half_len 默认值一致
HALF_W = 1.00
EVADE = 1.6              # 与 launch 里的 evade_clear 一致


class Fake(PedestrianAgent):
    """只为调用 _sector_dist 提供属性（**不调用** Node.__init__，不碰 rclpy）。"""

    def __init__(self, vehicles, fov_deg=140.0, rays=17):
        self.fov_half = math.radians(fov_deg) * 0.5
        self.fov_rays = rays
        self.body_hl = HALF_L
        self.body_hw = HALF_W
        self.move_th = 0.25
        self.vehicles = vehicles


def sect(vehicles, x=0.0, y=0.0, yaw=0.0, fov_deg=140.0, reach=8.0):
    return PedestrianAgent._sector_hit(Fake(vehicles, fov_deg), x, y, yaw, reach)[0]


def veh(x, y, yaw=0.0, spd=0.0):
    return {'v': (x, y, yaw, 0.0, spd)}


CASES = []


def case(name, d, blocked, want_d=None, tol=0.25):
    CASES.append((name, d, blocked, want_d, tol))


def main():
    # ---- 正前方：应当令行 ----
    case('正前方 3.5 m（车头朝 +x）', sect(veh(3.5, 0)), True, 3.5 - HALF_L)
    case('正前方 5.0 m（车头朝 +x）', sect(veh(5.0, 0)), False, 5.0 - HALF_L)
    case('正前方 5.0 m（车横置）', sect(veh(5.0, 0, math.pi / 2)), False,
         5.0 - HALF_W)

    # ---- 正后方：**这是本次修复的核心**。旧的圆形判据下 d_body = 4-2.2 = 1.8，
    #      近似触发；再近一点（3 m）就一定触发，行人在车尾站着不动。----
    case('正后方 4 m', sect(veh(-4.0, 0)), False, None)
    case('正后方 3 m', sect(veh(-3.0, 0)), False, None)
    case('左后方 150°/2.5 m', sect(veh(-2.17, -1.25)), False, None)

    # ---- 正侧方 3 m（横向间隙 3-1=2 m）：距离够远，不该停 ----
    case('正侧方 3 m', sect(veh(0.0, 3.0)), False, None)

    # ---- 贴着车身（横向间隙 0.2 m）：任何判据都该停 ----
    case('贴住车身（侧向间隙 0.2 m）', sect(veh(0.0, 1.2)), True, 0.2)
    case('行人在车身内', sect(veh(0.5, 0.0)), True, 0.0)

    # ---- fov=180° 等价于旧的圆形判据：正侧方 3 m 也应看到，但距离仍是 2 m ----
    case('fov=180° 侧方 3 m 可见', sect(veh(0.0, 3.0), fov_deg=180.0), False,
         3.0 - HALF_W)

    bad = 0
    for name, d, want_blocked, want_d, tol in CASES:
        got_blocked = d < EVADE
        ok = got_blocked == want_blocked
        if ok and want_d is not None:
            ok = abs(d - want_d) <= tol
        bad += 0 if ok else 1
        dist = 'inf' if d == float('inf') else f'{d:.3f}'
        want = (f'{"距 %.2f" % want_d if want_d is not None else "任意"}'
                f' / {"让行" if want_blocked else "通行"}')
        print(f'  [{"OK" if ok else "--"}] {name:<26} 距 {dist:<7}'
              f' {"让行" if got_blocked else "通行":<4} 期望 {want}')
    print(f'\n{len(CASES) - bad}/{len(CASES)} 通过（判据：距 < {EVADE} m 且在前方扇形内）')

    # ---- 运动标志：同样距离，车停了就不该再让（否则车等人、人等车，互锁）----
    rows = [
        ('前车驶来 1.3 m', veh(3.5, 0.0, spd=1.2), True),
        ('前车已停 1.3 m', veh(3.5, 0.0, spd=0.0), False),
        ('前车慢速 0.3 m/s', veh(3.5, 0.0, spd=0.3), True),
    ]
    print('\n运动标志（_sector_hit 的第二个返回值）')
    for name, v, want in rows:
        got = PedestrianAgent._sector_hit(Fake(v), 0.0, 0.0, 0.0, 8.0)[1]
        ok = got == want
        bad += 0 if ok else 1
        print(f'  [{"OK" if ok else "--"}] {name:<20} 在动={got} 期望 {want}')

    bad2 = ego_cases()
    return 1 if (bad or bad2) else 0


class FakeEgo:
    """只为调用 _ped_in_front 提供属性（不构造 ROS 节点）。"""

    def __init__(self, peds, pose, gear=1, near=2.5, fov=140.0):
        self.peds = peds
        self.pose = pose
        self._near = near
        self._fov = fov
        self.follower = types.SimpleNamespace(gear=gear)

    def get_parameter(self, name):
        if name == 'ped_fov':
            return types.SimpleNamespace(value=self._fov)
        return types.SimpleNamespace(value=self._near)


def ego_wait(peds, pose=(0.0, 0.0, 0.0), gear=1, near=2.5, fov=140.0):
    return ParkingPlanner._ped_in_front(
        FakeEgo(peds, pose, gear, near, fov), near)


def ego_cases():
    """自车侧：只有行驶方向前方扇形内的行人才让自车停车。"""
    rows = [
        ('车头正前 3.5 m', [(3.5, 0.0)], 1, True),
        ('车头前 6 m（超出 2.5 m）', [(6.0, 0.0)], 1, False),
        ('车侧 1.4 m（横向）', [(0.0, 1.4)], 1, False),
        ('车尾 2.6 m（前进时）', [(-2.6, 0.0)], 1, False),
        ('车尾 2.6 m（倒车时）', [(-2.6, 0.0)], -1, True),
        ('车头前 4 m（倒车时）', [(4.0, 0.0)], -1, False),
        ('贴住车身侧向 0.9 m', [(0.0, 0.9)], 1, True),
        ('斜前 45°/2 m', [(1.41, 1.41)], 1, True),
        ('斜后 135°/2 m', [(-1.41, 1.41)], 1, False),
    ]
    bad = 0
    print('\n自车侧 _ped_in_front（ped_yield=2.5 m，ped_fov=140°）')
    for name, peds, gear, want in rows:
        got = ego_wait(peds, gear=gear)
        ok = got == want
        bad += 0 if ok else 1
        print(f'  [{"OK" if ok else "--"}] {name:<24} {"停车等待" if got else "继续行驶":<6}'
              f' 期望 {"停车等待" if want else "继续行驶"}')
    print(f'\n{len(rows) - bad}/{len(rows)} 通过')
    return bad


if __name__ == '__main__':
    sys.exit(main())
