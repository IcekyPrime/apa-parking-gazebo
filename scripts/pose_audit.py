#!/usr/bin/env python3
"""自车位姿真值对照：把 `/vehicle_pose` 与 `/odom_gt` **逐帧**记下来，量出实际误差。

为什么需要它
------------
离线能把「规划 + 跟踪 + 执行器」算到 0.1 m 以内，真机却出现过 1 m 级的横向漂移。
差的那一项只能是**自车位姿本身的测量误差**，而原来的 `slam_eval` 只能给出一个
「首帧平移对齐后的 ATE RMSE」，它回答不了三个真正要紧的问题：

1. `/vehicle_pose` 相对真值**滞后多少**（ATE 对滞后不敏感，但滞后直接吃相位余量）；
2. 误差是**常值偏置**（可以被锚定/规划吸收）、**缓慢漂移**（会累积成横向发散），
   还是**零均值噪声**（只会让跟踪抖）；
3. 航向误差有多大 —— 入位弧上的反馈余量折算下来只有 1.4°（见清单 §6.23），
   航向误差一旦超过它，曲率指令就顶在限幅上退不出来。

本脚本因此不用固定采样率「各取最新一帧」，而是**两条话题各记各的时间戳**，
事后在任意滞后上重采样、重新做刚体配准，把「滞后 / 偏置 / 漂移」分开量。

用法
----
    # 采集（Ctrl-C 结束，收尾时写 <csv>_summary.txt）
    python3 scripts/pose_audit.py --csv /tmp/pose_E3.csv

    # 事后分析（可以反复跑，不用重开仿真）
    python3 scripts/pose_audit.py --analyze /tmp/pose_E3.csv

CSV 列：`src,t_recv,t_stamp,x,y,yaw`，`src` ∈ {gt, est}。
`t_stamp` 取消息 header 的仿真时间（两边都是 sim time）；`/vehicle_pose` 的 header
是**发布时刻**而不是 TF 的取用时刻，所以「它到底有多旧」只能靠滞后扫描量出来。
`t_recv` 是进程收包时刻（monotonic），header 不可用时兜底。
"""
import argparse
import bisect
import math
import os
import sys
import time


def wrap(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# --------------------------------------------------------------------------- 采集
def make_recorder(csv_path):
    """构造采集节点。`rclpy` 在这里才 import —— 这样 `--analyze` 在没有装 ROS 的
    环境里（或只 source 了工作区没 source 底层）也能跑。"""
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import Odometry
    from rclpy.node import Node

    class PoseAudit(Node):
        def __init__(self, path):
            super().__init__('pose_audit')
            self.path = os.path.expanduser(path)
            d = os.path.dirname(self.path)
            if d:
                os.makedirs(d, exist_ok=True)
            self.fp = open(self.path, 'w')
            self.fp.write('src,t_recv,t_stamp,x,y,yaw\n')
            self.n = {'gt': 0, 'est': 0}
            self.t0 = time.monotonic()
            self.create_subscription(PoseStamped, '/vehicle_pose', self.on_est, 40)
            self.create_subscription(Odometry, '/odom_gt', self.on_gt, 40)
            self.get_logger().info(
                f'开始记录 /vehicle_pose 与 /odom_gt → {self.path}（Ctrl-C 收尾）')

        def _row(self, src, stamp, x, y, yaw):
            t_recv = time.monotonic() - self.t0
            self.fp.write(f'{src},{t_recv:.4f},{stamp:.4f},'
                          f'{x:.4f},{y:.4f},{yaw:.5f}\n')
            self.n[src] += 1
            if (self.n['gt'] + self.n['est']) % 20 == 0:
                self.fp.flush()

        def on_gt(self, msg):
            p = msg.pose.pose.position
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._row('gt', st, p.x, p.y, yaw_of(msg.pose.pose.orientation))

        def on_est(self, msg):
            p = msg.pose.position
            st = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self._row('est', st, p.x, p.y, yaw_of(msg.pose.orientation))

        def finish(self):
            try:
                self.fp.flush()
                self.fp.close()
            except Exception:
                pass
            self.get_logger().info(f'记录结束：/odom_gt {self.n["gt"]} 帧，'
                                   f'/vehicle_pose {self.n["est"]} 帧'
                                   f' → {self.path}')

    return PoseAudit(csv_path)


# --------------------------------------------------------------------------- 分析
def load(path):
    gt, est = [], []
    with open(path) as f:
        for line in f:
            if not line or line[0] in '#s':
                continue
            p = line.strip().split(',')
            if len(p) < 6:
                continue
            src, t_recv, t_stamp = p[0], float(p[1]), float(p[2])
            x, y, yaw = float(p[3]), float(p[4]), float(p[5])
            # 有 header 时间就用 header，没有（0）退回收包时刻
            t = t_stamp if t_stamp > 1e-6 else t_recv
            (gt if src == 'gt' else est).append((t, x, y, yaw))
    gt.sort(key=lambda r: r[0])
    est.sort(key=lambda r: r[0])
    return gt, est


def interp(series, times, t):
    """在 series 上线性插值（时间轴 times 与 series 一一对应，已排序）。"""
    if not series or t < times[0] or t > times[-1]:
        return None
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return series[0][1:]
    if i >= len(times):
        return series[-1][1:]
    t0, t1 = times[i - 1], times[i]
    a, b = series[i - 1], series[i]
    r = 0.0 if t1 - t0 < 1e-9 else (t - t0) / (t1 - t0)
    return (a[1] + (b[1] - a[1]) * r,
            a[2] + (b[2] - a[2]) * r,
            wrap(a[3] + wrap(b[3] - a[3]) * r))


def rigid(u, v):
    """刚体配准：把 **真值 v** 变换到 **估计 u** 的坐标系，返回 (δ, tx, ty)。

    **旋转角只从航向通道取**（`est_yaw − gt_yaw` 的圆均值），不用 Procrustes。
    原因：map 帧的旋转偏置是个常值未知量，航向是两边都直接测到的，估计它跟轨迹
    形状无关、永远良态；而用位置做最小二乘时，**短窗口 + 低速**会退化 ——
    实测只取入位那 20 s 时 Procrustes 给出 δ=+6.7°，而全程给出 +0.07°，
    差了 6.6°，于是「航向残差 6.63°」全是配准自己造出来的假误差。
    旋转定死之后，平移就是 `mean(u) − R·mean(v)`，这是固定旋转下的解析最优解。
    """
    n = len(u)
    sy = sum(math.sin(wrap(a[2] - b[2])) for a, b in zip(u, v))
    cy = sum(math.cos(wrap(a[2] - b[2])) for a, b in zip(u, v))
    d = math.atan2(sy, cy) if (abs(sy) + abs(cy) > 1e-12) else 0.0
    nux = sum(p[0] for p in u) / n
    nuy = sum(p[1] for p in u) / n
    nvx = sum(p[0] for p in v) / n
    nvy = sum(p[1] for p in v) / n
    c, s = math.cos(d), math.sin(d)
    return d, nux - (c * nvx - s * nvy), nuy - (s * nvx + c * nvy)


def stats_at_lag(gt, est, e_t, lag):
    """在「估计滞后 lag 秒」的假设下配对，返回 (配准结果, 残差列表, 时间列表)。

    约定：`lag>0` 表示 `/vehicle_pose` 比真值**旧** `lag` 秒，即
    `est(t) ≈ gt(t − lag)`。要跟 `gt(t)` 比就得取 `est(t + lag)`。
    第一版写成 `t − lag`，自检（注入 +0.35 s）就量出 −0.28 s——符号整个反了，
    是自检把它抓出来的。
    """
    pairs, ts = [], []
    for g in gt:
        e = interp(est, e_t, g[0] + lag)
        if e is None:
            continue
        pairs.append((e, (g[1], g[2], g[3])))
        ts.append(g[0])
    if len(pairs) < 10:
        return None, [], []
    d, tx, ty = rigid([p[0] for p in pairs], [p[1] for p in pairs])
    c, s = math.cos(d), math.sin(d)
    res = []
    for (e, g) in pairs:
        gx = c * g[0] - s * g[1] + tx
        gy = s * g[0] + c * g[1] + ty
        ex, ey = e[0] - gx, e[1] - gy
        eyaw = wrap(e[2] - wrap(g[2] + d))
        res.append((ex, ey, eyaw))
    return (d, tx, ty), res, ts


def derived(series):
    """由位姿序列导出 **速度** 与 **偏航率**，两者都对刚体变换免疫。

    为什么必须多这两个量：位姿误差是「滞后」还是「map 帧转了个角」在**常曲率
    轨迹上是简并的** —— 沿圆弧整体平移/绕圆心旋转，跟「晚一点」在几何上完全等价，
    配准残差怎么拟合都能吃掉。所以只靠「配准后残差最小」去挑滞后，会得到一条
    平坦的曲线（实测自检里 τ=0~1.4 s 的残差只从 0.073 变到 0.079，只有真正
    对齐的那一格掉到 0.050）。
    速度只跟平移的一阶差分有关、偏航率只跟旋转的一阶差分有关，平移和常值旋转
    都被差分掉了 —— 只有**时间错位**留在里面。三个估计量一致，才叫「纯延迟」。
    """
    out = []
    for k in range(1, len(series)):
        dt = series[k][0] - series[k - 1][0]
        if dt < 1e-6:
            continue
        a, b = series[k - 1], series[k]
        v = math.hypot(b[1] - a[1], b[2] - a[2]) / dt
        w = wrap(b[3] - a[3]) / dt
        out.append(((a[0] + b[0]) * 0.5, v, w))
    return out


def sample(d, times, t):
    if t < times[0] or t > times[-1]:
        return None
    i = bisect.bisect_left(times, t)
    if i <= 0:
        return d[0][1:]
    if i >= len(times):
        return d[-1][1:]
    t0, t1 = times[i - 1], times[i]
    a, b = d[i - 1], d[i]
    r = 0.0 if t1 - t0 < 1e-9 else (t - t0) / (t1 - t0)
    return (a[1] + (b[1] - a[1]) * r, a[2] + (b[2] - a[2]) * r)


def boxcar(rows, win):
    """按时间宽度的滑动平均（rows: (t, v, w)）。

    速度是由位置差分出来的，位置噪声被 `1/dt` 放大 —— 5 cm / 20 Hz 就是
    ±1.4 m/s 的假速度，而真速度只有 0.45 m/s，不做平滑的话相关性全是噪声。
    航向差分的噪声小一个量级，不需要（平滑反而把滞后抹平）。
    """
    if win <= 0.0 or not rows:
        return rows
    out = []
    j0 = 0
    for k, r in enumerate(rows):
        while rows[j0][0] < r[0] - win * 0.5:
            j0 += 1
        j1 = k
        while j1 + 1 < len(rows) and rows[j1 + 1][0] <= r[0] + win * 0.5:
            j1 += 1
        n = j1 - j0 + 1
        out.append((r[0],
                    sum(x[1] for x in rows[j0:j1 + 1]) / n,
                    sum(x[2] for x in rows[j0:j1 + 1]) / n))
    return out


def lag_by_signal(gt, est, which=0):
    """用速度（which=0）或偏航率（which=1）的相关性找滞后。

    判据用 **归一化互相关**（等价于最小二乘的归一化形式），对幅值尺度不敏感。
    返回 (最佳滞后, 该滞后下的相关系数, 曲线)。
    """
    gd = derived(gt)
    ed = derived(est)
    if which == 0:
        gd, ed = boxcar(gd, 0.6), boxcar(ed, 0.6)
    if len(gd) < 20 or len(ed) < 20:
        return None, 0.0, []
    ed_t = [r[0] for r in ed]
    curve = []
    for i in range(-30, 76):                 # -0.6 ~ +1.5 s，步长 0.02
        lag = i * 0.02
        xs, ys = [], []
        for g in gd:
            e = sample(ed, ed_t, g[0] + lag)
            if e is None:
                continue
            xs.append(g[1 + which])
            ys.append(e[which])
        if len(xs) < 20:
            continue
        mx = sum(xs) / len(xs)
        my = sum(ys) / len(ys)
        sxy = sum((a - mx) * (b - my) for a, b in zip(xs, ys))
        sxx = sum((a - mx) ** 2 for a in xs)
        syy = sum((b - my) ** 2 for b in ys)
        den = math.sqrt(sxx * syy)
        curve.append((round(lag, 2), sxy / den if den > 1e-12 else 0.0))
    if not curve:
        return None, 0.0, []
    b = max(curve, key=lambda p: p[1])
    return b[0], b[1], curve


def rms(res):
    if not res:
        return float('nan')
    return math.sqrt(sum(r[0] * r[0] + r[1] * r[1] for r in res) / len(res))


def analyze(path, t_from=None, t_to=None):
    gt, est = load(path)
    out = []
    if len(gt) < 10 or len(est) < 10:
        return (f'数据不足：/odom_gt {len(gt)} 帧、/vehicle_pose {len(est)} 帧\n'
                f'（采集时是不是没起仿真？路径 {path}）')
    # 可选的时段裁剪（相对于第一帧 /odom_gt 的秒数）。入位段只占整趟的最后几十秒，
    # 而「巡航 1.4 m/s」那一大段会把滞后估计拉向它自己 —— 要专门看入位就得裁。
    if t_from is not None or t_to is not None:
        base = gt[0][0]
        lo = base + (t_from or 0.0)
        hi = base + (t_to if t_to is not None else 1e9)
        gt = [r for r in gt if lo <= r[0] <= hi]
        est = [r for r in est if lo <= r[0] <= hi]
        out.append(f'时段裁剪：t∈[{t_from or 0.0:.1f}, '
                   f'{t_to if t_to is not None else float("nan"):.1f}] s'
                   f'（相对首帧）→ /odom_gt {len(gt)} 帧、{len(est)} 帧')
        if len(gt) < 10 or len(est) < 10:
            return '\n'.join(out) + '\n该时段数据不足'
    gt_t = [r[0] for r in gt]
    e_t = [r[0] for r in est]
    dur = gt_t[-1] - gt_t[0]
    rate_g = len(gt) / max(dur, 1e-6)
    rate_e = len(est) / max(dur, 1e-6)
    overlap0 = max(gt_t[0], e_t[0])
    overlap1 = min(gt_t[-1], e_t[-1])
    out.append(f'样本：/odom_gt {len(gt)} 帧（{rate_g:.1f} Hz）、'
               f'/vehicle_pose {len(est)} 帧（{rate_e:.1f} Hz），'
               f'时长 {dur:.1f} s，重叠 {max(0.0, overlap1 - overlap0):.1f} s')

    # ---- 滞后扫描：找让残差最小的「估计滞后」
    grid = [round(-0.30 + 0.02 * i, 2) for i in range(int((1.50 + 0.30) / 0.02) + 1)]
    best = None
    curve = []
    for lag in grid:
        rig, res, ts = stats_at_lag(gt, est, e_t, lag)
        if rig is None:
            continue
        r = rms(res)
        curve.append((lag, r, rig, res, ts))
        if best is None or r < best[1]:
            best = (lag, r, rig, res, ts)
    if best is None:
        out.append('两路数据在时间上没有重叠，无法配对')
        return '\n'.join(out)
    lag, r_best, (d, tx, ty), res, ts = best
    zero = next((c for c in curve if abs(c[0]) < 1e-9), None)
    out.append(f'map 帧相对世界帧：旋转 δ={math.degrees(d):+.2f}°，'
               f'平移 ({tx:+.2f}, {ty:+.2f}) m')
    out.append(f'最佳滞后 τ*={lag:+.2f} s → 位置 RMSE {r_best:.3f} m'
               + (f'（τ=0 时 {zero[1]:.3f} m）' if zero else ''))

    # 滞后曲线（只打 τ≥0 的那半，负滞后基本没有物理意义）
    out.append('\n滞后扫描（估计滞后 τ / 位置 RMSE / 航向残差 RMS）：')
    for c in curve:
        if c[0] < -1e-9:
            continue
        if abs((c[0] * 100) % 20) > 1e-6 and abs(c[0] - lag) > 1e-9:
            continue                        # 每 0.2 s 一行，外加最佳点
        yaw_rms = math.sqrt(sum(r[2] * r[2] for r in c[3]) / len(c[3]))
        mark = ' ← 位置最优' if abs(c[0] - lag) < 1e-9 else ''
        out.append(f'  τ={c[0]:+.2f}s  {c[1]:.3f} m  '
                   f'{math.degrees(yaw_rms):5.2f}°{mark}')

    # ---- 两个对刚体变换免疫的滞后估计（速度 / 偏航率的一阶差分）
    tau_v, cv, _ = lag_by_signal(gt, est, 0)
    tau_w, cw, _ = lag_by_signal(gt, est, 1)
    out.append('\n三个互相独立的滞后估计：')
    out.append(f'  ① 位置配准残差最小：τ={lag:+.2f} s')
    # 相关系数太低说明这个信号**本身没有变化**（比如全程匀速，速度差分是常数），
    # 那它的「最佳滞后」就是噪声里挑出来的，不能当证据。
    if tau_v is not None and cv >= 0.30:
        out.append(f'  ② 速度互相关最大：  τ={tau_v:+.2f} s（r={cv:.3f}）')
    else:
        out.append(f'  ② 速度：不可用（r={cv:.3f}，全程速度变化太小，判别力不足）')
    if tau_w is not None and cw >= 0.30:
        out.append(f'  ③ 偏航率互相关最大：τ={tau_w:+.2f} s（r={cw:.3f}）')
    else:
        out.append(f'  ③ 偏航率：不可用（r={cw:.3f}，全程几乎没转弯）')
    ests = [x for x in (tau_v if cv >= 0.30 else None,
                        tau_w if cw >= 0.30 else None) if x is not None]
    if ests:
        ests.append(lag)
        spread = max(ests) - min(ests)
        if spread <= 0.10:
            out.append(f'  {len(ests)} 个估计相差 ≤{spread * 1000:.0f} ms'
                       f' → 可以按**纯延迟**处理'
                       f'（建模就是 `POSE_LAT={sum(ests) / len(ests):.2f}`）')
        else:
            out.append(f'  {len(ests)} 个估计最大相差 {spread:.2f} s'
                       f' → **不是纯延迟**：位置与航向的误差结构不同，得分开查')

    # ---- 残差分解（在 τ* 下）
    n = len(res)
    err = [math.hypot(r[0], r[1]) for r in res]
    err_sorted = sorted(err)
    yaw_rms = math.sqrt(sum(r[2] * r[2] for r in res) / n)
    mean_ex = sum(r[0] for r in res) / n
    mean_ey = sum(r[1] for r in res) / n
    out.append(f'\n在 τ* 下的残差：位置 RMSE {rms(res):.3f} m，'
               f'均值 ({mean_ex:+.3f}, {mean_ey:+.3f}) m，'
               f'P95 {err_sorted[int(0.95 * (n - 1))]:.3f} m，'
               f'最大 {err_sorted[-1]:.3f} m；航向 RMS {math.degrees(yaw_rms):.2f}°')

    # ---- 按 10 s 分桶：区分「常值偏置」还是「累积漂移」
    t0 = ts[0]
    buckets = {}
    for (r, t) in zip(res, ts):
        k = int((t - t0) // 10.0)
        buckets.setdefault(k, []).append(r)
    out.append('\n分时段（10 s 一桶）—— 看误差是偏置还是漂移：')
    for k in sorted(buckets):
        rs = buckets[k]
        ex = sum(r[0] for r in rs) / len(rs)
        ey = sum(r[1] for r in rs) / len(rs)
        yy = math.sqrt(sum(r[2] * r[2] for r in rs) / len(rs))
        out.append(f'  t={t0 + 10 * k - t0:5.0f}~{10 * k + 10:3.0f}s  n={len(rs):4d}  '
                   f'均值偏差=({ex:+.3f},{ey:+.3f}) m  |e|_rms={rms(rs):.3f} m  '
                   f'航向RMS={math.degrees(yy):5.2f}°')
    return '\n'.join(out)


# --------------------------------------------------------------------------- 入口
def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument('--csv', default='/tmp/pose_audit.csv')
    ap.add_argument('--analyze', default='')
    ap.add_argument('--from', dest='t_from', type=float, default=None,
                    help='只分析「首帧之后第几秒」开始的一段（相对首帧 /odom_gt）')
    ap.add_argument('--to', dest='t_to', type=float, default=None)
    args = ap.parse_args(argv)

    if args.analyze:
        print(analyze(os.path.expanduser(args.analyze), args.t_from, args.t_to))
        return 0

    import rclpy
    rclpy.init()
    node = make_recorder(args.csv)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.finish()
    node.destroy_node()
    try:
        rclpy.shutdown()
    except Exception:
        pass
    return 0


if __name__ == '__main__':
    sys.exit(main())
