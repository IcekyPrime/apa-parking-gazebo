#!/usr/bin/env python3
"""生成合成轨迹数据集，用于训练 Transformer 轨迹预测模型。

运动模式：直行(straight)、转弯(turn)、穿行(cross)、驶入车位(park_in)、驶出车位(park_out)、横穿(jaywalk)、静止起步(stand_start)、急停(sudden_stop)、静止(stand_still)、折返(reversal)
目标类型：行人(pedestrian)、车辆(vehicle)

每条样本：
  - past:   过去 8 帧 [x, y, vx, vy]（x/y 已相对"当前帧"归一化）
  - future: 未来 5 步 [x, y]（同样相对当前帧）
  - type:   0=pedestrian, 1=vehicle

泊车场景补充（关键）：原有 3 种模式都是恒定速度，无法描述"减速停车""低速犹豫"和"静止起停"。
  - park_in：车辆边转弯边减速，末速度接近 0（泊车机动）
  - park_out：车辆低速起步、加速转弯驶出车位（泊车机动）
  - jaywalk：行人低速横穿停车场，中途减速犹豫再继续
  - stand_start：静止行人突然起步（覆盖"前方站着不动的人突然窜出"这一最危险情形）
  - sudden_stop：行人/车辆运动中急停
  - stand_still：行人/车辆全程静止（覆盖"站住不动/停放"目标，消除模型对静止输入的偏置）
  - reversal：行人/车辆匀速走到某时刻瞬间掉头（覆盖 Gazebo 横穿行人的端点折返，速度方向瞬时翻转）

默认 14 种模式/类型组合 × 350 条 = 4900 条样本（满足 >2000 要求）。
保存为 trajectory_dataset.npz。
"""

import argparse
from pathlib import Path

import numpy as np

DT = 0.4                    # 帧间隔（秒）
PAST = 8                    # 过去帧数
FUTURE = 5                  # 未来步数
STEPS = PAST + FUTURE       # 总步数 13


def simulate(rng, pattern, v0, heading0, p0=(0.0, 0.0), dt=DT, steps=STEPS):
    """从运动模型生成 (steps, 4) 的 [x, y, vx, vy] 序列。"""
    x, y = p0
    heading = heading0
    traj = np.zeros((steps, 4))

    omega = 0.0             # 恒定角速度（弧度/秒）
    omega_profile = None    # 逐帧角速度序列（弧度/秒）
    decel_rate = 0.0        # park_in: 减速速率（1.0 = 末速度 0）
    accel_rate = 0.0        # park_out: 起步加速速率（>1 则中途达到目标速度后匀速）
    dip = 0.0               # jaywalk: 犹豫段最低速度比例
    dip_center = 0.5        # jaywalk: 犹豫时刻（0~1 时间比例）
    start_frac = 0.0        # stand_start: 起步时刻（0~1 时间比例）
    stop_frac = 0.0         # sudden_stop: 急停时刻（0~1 时间比例）
    rev_frac = 0.5          # reversal: 折返时刻（0~1 时间比例）

    if pattern == 'turn':
        omega = rng.uniform(0.4, 1.0) * rng.choice([-1.0, 1.0])
    elif pattern == 'cross':
        # 穿行/变道：航向呈正弦摆动（S 形）
        omega_profile = 0.6 * np.sin(np.linspace(0.0, np.pi, steps)) * rng.choice([-1.0, 1.0])
    elif pattern == 'park_in':
        # 车辆驶入车位：边转弯边减速，末速度接近 0（泊车机动）
        omega = rng.uniform(0.3, 0.9) * rng.choice([-1.0, 1.0])
        decel_rate = rng.uniform(0.7, 1.0)
    elif pattern == 'park_out':
        # 车辆驶出车位：低速起步、加速转弯驶出车位
        omega = rng.uniform(0.2, 0.8) * rng.choice([-1.0, 1.0])
        accel_rate = rng.uniform(0.9, 1.5)
    elif pattern == 'jaywalk':
        # 行人横穿停车场：低速直线，中途减速犹豫再继续
        dip = rng.uniform(0.2, 0.5)
        dip_center = rng.uniform(0.35, 0.65)
    elif pattern == 'stand_start':
        # 静止→突然起步：前段速度为 0，到 start_frac 附近快速加速到 v0
        start_frac = rng.uniform(0.35, 0.55)
    elif pattern == 'sudden_stop':
        # 运动中急停：前段匀速 v0，到 stop_frac 附近快速减速到 0
        stop_frac = rng.uniform(0.45, 0.65)
    elif pattern == 'reversal':
        # 折返：沿航向匀速走到 rev_frac 后瞬间掉头（匹配 Gazebo 横穿行人的端点折返）
        rev_frac = rng.uniform(0.3, 0.85)

    for t in range(steps):
        s = t / (steps - 1)                     # 0..1 时间比例
        v = v0
        if pattern == 'park_in':
            v = v0 * max(0.0, 1.0 - decel_rate * s)
        elif pattern == 'park_out':
            v = v0 * min(1.0, accel_rate * s)
        elif pattern == 'jaywalk':
            # 犹豫段 dip_center 附近速度降到 dip*v0，两端恢复 v0（平滑减速）
            v = v0 * (1.0 - (1.0 - dip) * np.exp(-((s - dip_center) ** 2) / 0.02))
        elif pattern == 'stand_start':
            v = 0.0 if s < start_frac else v0 * min(1.0, (s - start_frac) / 0.15)
        elif pattern == 'sudden_stop':
            v = v0 if s < stop_frac else v0 * max(0.0, 1.0 - (s - stop_frac) / 0.12)
        elif pattern == 'reversal':
            v = v0 if s < rev_frac else -v0
        elif pattern == 'stand_still':
            v = 0.0
        v *= 1.0 + rng.normal(0.0, 0.05)        # 速度轻微噪声

        vx = v * np.cos(heading)
        vy = v * np.sin(heading)
        traj[t] = [x, y, vx, vy]

        # 更新状态
        if omega_profile is not None:
            heading += omega_profile[t] * dt
        else:
            heading += omega * dt
        heading += rng.normal(0.0, 0.03)        # 航向噪声
        x += vx * dt
        y += vy * dt

    return traj


def smooth_velocity(vel, lam=0.5):
    """一阶低通（指数平滑）速度：近似在线检测节点 alpha-beta 滤波器的速度滞后/平滑。

    当前【未使用】——这是"训练-在线速度对齐"实验的遗留工具。那次实验结论：
    在线 alpha-beta 速度残差 0.24m/s，远低于向后差分的 0.59m/s，但 alpha-beta
    直接上线会引入加减速段滞后导致 ADE 升到 0.69m，故最终放弃速度对齐、保持
    真值速度训练 + 在线向后差分。保留此函数仅作记录，若未来要重做速度对齐可复用。
    """
    out = np.zeros_like(vel)
    out[0] = vel[0]
    for t in range(1, len(vel)):
        out[t] = lam * out[t - 1] + (1.0 - lam) * vel[t]
    return out


def make_sample(rng, pattern, type_id):
    """生成一条样本，返回 past(8,4), future(5,2), type_id。

    位置/未来帧保持干净真值（相对当前帧），模型学习"从带噪过去预测真实未来"。
    速度用 simulate 直接给出的真值 vx/vy——这是当前在线 checkpoint 的训练分布。
    """
    if type_id == 0:      # pedestrian
        v0 = rng.uniform(0.8, 1.6)
    else:                 # vehicle
        v0 = rng.uniform(2.0, 6.0)

    if pattern == 'park_in':
        v0 = rng.uniform(1.0, 3.0)      # 泊车低速
    elif pattern == 'jaywalk':
        v0 = rng.uniform(0.5, 1.3)      # 横穿行人低速
    elif pattern == 'stand_start':
        v0 = rng.uniform(1.0, 2.0)      # 静止起步通常更突然、稍快

    heading0 = rng.uniform(0.0, 2.0 * np.pi)
    p0 = (rng.uniform(-20.0, 20.0), rng.uniform(-12.0, 12.0))

    traj = simulate(rng, pattern, v0, heading0, p0)   # (13, 4) 干净真值

    # 位置干净，速度用真值（simulate 直接给出的 vx/vy）。
    ref = traj[PAST - 1, 0:2]

    past = np.zeros((PAST, 4), dtype=np.float32)
    past[:, 0:2] = traj[:PAST, 0:2] - ref
    past[:, 2:4] = traj[:PAST, 2:4]   # 真值速度

    future = (traj[PAST:STEPS, 0:2] - ref).astype(np.float32)
    return past, future, type_id


# 每种运动模式适用的目标类型（驶入车位是车辆、横穿是行人，其余两类都适用）
COMBOS = [
    ('straight', 0), ('straight', 1),
    ('turn', 0), ('turn', 1),
    ('cross', 0), ('cross', 1),
    ('park_in', 1),     # 车辆驶入车位
    ('park_out', 1),    # 车辆驶出车位
    ('jaywalk', 0),     # 行人横穿停车场
    ('stand_start', 0), # 行人静止→突然起步
    ('sudden_stop', 0), # 行人急停
    ('sudden_stop', 1), # 车辆急刹
    ('stand_still', 0), # 行人全程静止
    ('stand_still', 1), # 车辆全程静止（停放）
]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--out', type=Path,
                        default=Path(__file__).parent / 'trajectory_dataset.npz')
    parser.add_argument('--num', type=int, default=350,
                        help='每种模式/类型组合的样本数（总数 = num × 14）')
    parser.add_argument('--seed', type=int, default=42)
    args = parser.parse_args()

    rng = np.random.default_rng(args.seed)

    past_list, future_list, type_list = [], [], []
    for pattern, type_id in COMBOS:
        for _ in range(args.num):
            past, future, tid = make_sample(rng, pattern, type_id)
            past_list.append(past)
            future_list.append(future)
            type_list.append(tid)

    past = np.stack(past_list)          # (N, 8, 4)
    future = np.stack(future_list)      # (N, 5, 2)
    type_arr = np.array(type_list, dtype=np.int64)   # (N,)

    np.savez_compressed(args.out, past=past, future=future, type=type_arr)
    print(f'生成 {past.shape[0]} 条样本 -> {args.out}')
    print(f'  past:   {past.shape}')
    print(f'  future: {future.shape}')
    print(f'  type:   {type_arr.shape}, 分布 {np.bincount(type_arr)}')


if __name__ == '__main__':
    main()
