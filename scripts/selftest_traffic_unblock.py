#!/usr/bin/env python3
"""干扰车防死锁状态机的单测（起真 rclpy 节点，但不起 Gazebo）。

背景：真机 E2 回归实测到自车与干扰车互相让行、永久僵持到超时（自车横在
过道里等干扰车先走，干扰车的前方走廊里有自车也在等）。path_agent 现在
加了「让行超 `unblock_wait`(12 s) 且挡路车位移 < 0.3 m → 身后安全时主动
倒退 `unblock_back`(1 m)」的出口，与行人 YieldPolicy「等待→退让→再等待」
同一套思路。这里把状态机的每条边界都过一遍：

  1. 僵持成立（对方横停且不动）→ 进入倒退、发布负速；
  2. 对方在挪（位移 > 0.3 m）→ 继续等，不倒退；
  3. 身后有车 → 不倒退；
  4. 行人挡路 → 不倒退（行人侧有自己的 YieldPolicy）；
  5. 退够 1 m → 停止、发零速、重新计时；
  6. 倒退次数用满 `unblock_max`(3) → 第 4 次不再倒；
  7. 路清后真前进 > 3 m → 次数还回来。

用法：source /opt/ros/jazzy/setup.bash && python3 scripts/selftest_traffic_unblock.py
"""
import math
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'scene_agents'))

import rclpy  # noqa: E402
from rclpy.duration import Duration  # noqa: E402

from scene_agents.path_agent import PathAgent  # noqa: E402


class FakePub:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


SQUARE = [(0.0, 0.0), (40.0, 0.0), (40.0, 40.0), (0.0, 40.0)]


def make_node():
    node = PathAgent('unblock_test', SQUARE)
    node._placed = True          # 跳过随机落点（那要 Gazebo 的 set_pose 服务）
    node.pub = FakePub()
    return node


def age_blocker(node, sec):
    """把本次阻塞的开始时刻往回拨 sec 秒，模拟「已经让行这么久」。"""
    node._blk_t0 = node.get_clock().now() - Duration(seconds=sec)
    if node._blk_ref is None:
        node._blk_ref = node._blocker


def main():
    rclpy.init()
    node = make_node()
    bad = 0

    def check(name, got, want):
        nonlocal bad
        ok = (got == want)
        bad += 0 if ok else 1
        print(f'  [{"OK" if ok else "NG"}] {name:30s} '
              f'got={got!r}  want={want!r}')

    D = math.radians

    # ---- 1. 僵持：对方横停在前方且不动 → 让行 13 s 后进入倒退 ----
    node.pose = (0.0, 0.0, 0.0)
    node.others = {'ego': (6.0, 0.0, D(90))}
    node.tick()
    check('横停车在前方 → blocked', node.blocked, True)
    check('blocker 已记录', node._blocker, (6.0, 0.0))
    age_blocker(node, 13.0)
    node.tick()
    check('僵持 13 s → 进入倒退', node._back_from is not None, True)
    node.tick()                                  # 倒退执行拍
    check('倒退中发布负速', node.pub.last.linear.x < 0.0, True)
    node._back_from = None
    node._back_n = 0

    # ---- 2. 对方在挪（位移 > 0.3 m）→ 不倒退 ----
    node.blocked = False
    node._blk_t0 = None
    node._blk_ref = None
    node.pose = (0.0, 0.0, 0.0)
    node.others = {'ego': (6.0, 0.0, D(90))}
    node.tick()
    age_blocker(node, 13.0)
    node.others['ego'] = (6.8, 0.0, D(90))       # 挪了 0.8 m
    node.tick()
    check('对方挪了 0.8 m → 不倒退', node._back_from is None, True)

    # ---- 3. 身后有车 → 不倒退 ----
    node.blocked = False
    node._blk_t0 = None
    node._blk_ref = None
    node.pose = (0.0, 0.0, 0.0)
    node.others = {'ego': (6.0, 0.0, D(90)), 'rear': (-4.5, 0.0, 0.0)}
    node.tick()
    age_blocker(node, 13.0)
    node.tick()
    check('身后 4.5 m 有车 → 不倒退', node._back_from is None, True)

    # ---- 4. 行人挡路 → 不倒退（行人自己有 YieldPolicy）----
    node.blocked = False
    node._blk_t0 = None
    node._blk_ref = None
    node.others = {'rear': (-20.0, 0.0, 0.0)}
    node.peds = [(5.0, 0.0)]
    node.pose = (0.0, 0.0, 0.0)
    node.tick()
    check('行人挡路 → blocked', node.blocked, True)
    age_blocker(node, 13.0)
    node.tick()
    check('行人挡路 → 不倒退', node._back_from is None, True)
    node.peds = []

    # ---- 5. 倒退够 1 m → 停、发零速、重新计时 ----
    node.blocked = False
    node._blk_t0 = None
    node._blk_ref = None
    node.others = {'ego': (6.0, 0.0, D(90))}
    node.pose = (0.0, 0.0, 0.0)
    node.tick()
    age_blocker(node, 13.0)
    node.tick()
    check('进入倒退（前置）', node._back_from is not None, True)
    node.pose = (-1.2, 0.0, 0.0)                # 已退 1.2 m
    node.tick()
    check('退够 1 m → 停止倒退', node._back_from is None, True)
    check('退够后发零速', (node.pub.last.linear.x, node.pub.last.angular.z),
          (0.0, 0.0))
    check('倒退后重新计时', node._blk_t0 is not None, True)
    check('倒退计数 = 1', node._back_n, 1)
    check('锚点 = 倒退终点', node._yield_anchor, (-1.2, 0.0))

    # ---- 6. 次数上限：3 次用完 → 第 4 次不倒 ----
    node._back_n = 3
    age_blocker(node, 13.0)
    node.tick()
    check('3 次用完 → 不再倒退', node._back_from is None, True)

    # ---- 7. 路清后真前进 > 3 m → 次数还回来 ----
    node.others = {}                            # 路清了
    node.pose = (3.5, 0.0, 0.0)                 # 离锚点 (-1.2) 4.7 m
    node.tick()
    check('前进 > 3 m → 计数复位', node._back_n, 0)

    node.destroy_node()
    rclpy.shutdown()
    print(f'\n{"全部通过" if bad == 0 else f"失败 {bad} 条"}')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
