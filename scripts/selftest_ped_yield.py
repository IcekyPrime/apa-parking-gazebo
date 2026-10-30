#!/usr/bin/env python3
"""行人「等待 → 主动退让 → 再等待 ≥10 s → 再尝试前进」状态机的离线单测。

背景（用户截图里的「堵车」）：旧逻辑是「每帧被挡就退一小步、挡一解除就往前走
一步」，人被挡在车前时会在原地**一格格地蠕动**，车一直过不去。现在改成显式的
四段状态机（`pedestrian_agent.YieldPolicy`）。

这个测试把状态机单独跑 40 s（障碍一直存在 = 车停在人前面不动），断言：

1. 第一段 WAIT 的时长等于 ``wait_before_back``；
2. BACK 段必须**一次退够** ``back_dist``，而且期间不出现前进；
3. HOLD 段（第二次等待）**≥ 10 s**，且期间一步都不许动；
4. 四段顺序是 WAIT → BACK → HOLD → WALK；
5. 障碍提前消失时：在 WAIT 段要立刻恢复行走（不必真让），
   在 HOLD 段也要**站满** 10 s（不许提前走，否则让给车的时间不够）；
6. 后退预算耗尽时不再后退，直接进入 HOLD。

用法：`python3 scripts/selftest_ped_yield.py`
"""
import os
import sys

WS = os.path.expanduser(os.environ.get('APA_WS', '~/apa_ws'))
sys.path.insert(0, os.path.join(WS, 'src', 'scene_agents'))
sys.path.insert(0, os.path.join(WS, 'src', 'parking_planner'))

from scene_agents.pedestrian_agent import YieldPolicy   # noqa: E402

WALK, WAIT, BACK, HOLD = (YieldPolicy.WALK, YieldPolicy.WAIT,
                          YieldPolicy.BACK, YieldPolicy.HOLD)
DT = 0.1                      # 与 pedestrian_agent 的默认 rate=10 Hz 一致
KW = dict(walk_speed=1.1, back_speed=0.6,
          wait_before_back=2.5, back_dist=2.0, hold_after_back=10.5)


def run(blocked_fn, t_end=40.0, budget=1e9, kw=None):
    """跑一段仿真，返回 (分段列表, 逐帧轨迹)。

    分段列表元素：(mode, 时长 s, 该段累计带符号位移 m)。
    """
    p = YieldPolicy(**(kw or KW))
    t = 0.0
    segs = [[p.mode, 0.0, 0.0]]
    trace = []
    while t < t_end - 1e-9:
        ds = p.sample(blocked_fn(t), DT, budget_left=budget)
        segs[-1][2] += ds
        trace.append((round(t, 2), p.mode, round(ds, 4)))
        if p.mode != segs[-1][0]:
            segs.append([p.mode, t, 0.0])
        t += DT
    out = []
    for i, (m, t0, dsv) in enumerate(segs):
        t1 = segs[i + 1][1] if i + 1 < len(segs) else t
        out.append((m, t1 - t0, dsv))
    return out, trace


CASES = []


def case(name, ok, info=''):
    CASES.append((name, ok, info))


def seg_at(segs, mode, nth=0):
    hit = [s for s in segs if s[0] == mode]
    return hit[nth] if len(hit) > nth else None


def main():
    # ---------- 1. 障碍一直存在：完整的四段 ----------
    segs, trace = run(lambda t: True)
    seq = [s[0] for s in segs]
    case('状态顺序 WAIT→BACK→HOLD→WALK', seq[:5] == [WALK, WAIT, BACK, HOLD, WALK],
         ' -> '.join(seq[:5]))
    w = seg_at(segs, WAIT)
    b = seg_at(segs, BACK)
    h = seg_at(segs, HOLD)
    case(f'第一段 WAIT ≈ {KW["wait_before_back"]} s',
         w is not None and abs(w[1] - KW['wait_before_back']) < 0.2,
         f'{w[1]:.1f} s' if w else '无')
    case(f'BACK 一次退够 {KW["back_dist"]} m',
         b is not None and abs(-b[2] - KW['back_dist']) < 0.05,
         f'{-b[2]:.2f} m' if b else '无')
    case('HOLD 每段 ≥ 10 s', h is not None and h[1] >= 10.0,
         f'{h[1]:.1f} s' if h else '无')

    # ---------- 2. 不许蠕动 ----------
    bad = [x for x in trace
           if (x[1] in (WAIT, BACK) and x[2] > 1e-9)
           or (x[1] == HOLD and abs(x[2]) > 1e-9)]
    case('WAIT/BACK/HOLD 段内不出现前进（不蠕动）', not bad, f'{len(bad)} 帧违规')

    # ---------- 3. 障碍在 WAIT 期间消失 → 立刻恢复行走 ----------
    segs, trace = run(lambda t: t < 1.0)
    seq = [s[0] for s in segs]
    walked = sum(s[2] for s in segs if s[0] == WALK)
    case('障碍 1 s 就消失 → 不退让、直接继续走',
         seq[:2] == [WALK, WAIT] and BACK not in seq and walked > 3.0,
         f'{seq[:4]}，走了 {walked:.1f} m')

    # ---------- 4. 障碍在 HOLD 期间消失 → 仍要站满 10 s ----------
    segs, trace = run(lambda t: t < 6.0)      # 6 s 时消失（此时已在 BACK/HOLD）
    h = seg_at(segs, HOLD)
    case('障碍在 HOLD 期间消失 → 仍站满 10 s 才走',
         h is not None and h[1] >= 10.0, f'{h[1]:.1f} s' if h else '无')

    # ---------- 5. 预算耗尽 → 不后退，直接进入 HOLD ----------
    segs, trace = run(lambda t: True, budget=0.0)
    seq = [s[0] for s in segs]
    backed = sum(abs(s[2]) for s in segs if s[0] == BACK)
    case('后退预算为 0 → 不后退、直接站住等',
         backed < 1e-9 and HOLD in seq, f'后退 {backed:.2f} m')

    bad = 0
    for name, ok, info in CASES:
        bad += 0 if ok else 1
        print(f'  [{"OK" if ok else "--"}] {name:<38} {info}')
    print(f'\n{len(CASES) - bad}/{len(CASES)} 通过'
          f'（被挡后须：等 {KW["wait_before_back"]} s → 退 {KW["back_dist"]} m '
          f'→ 再等 ≥10 s → 再前进）')
    return 1 if bad else 0


if __name__ == '__main__':
    sys.exit(main())
