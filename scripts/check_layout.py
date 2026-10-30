#!/usr/bin/env python3
"""一致性校验：先验车位布局 yaml ↔ Gazebo world，并检查每个车位的入位位姿是否可通行。

为什么需要它
------------
`parking_planner/config/lot_layout.yaml` 不是手写配置，而是 `make_layout.py`
从 `city_parking_lot.sdf` 生成的**产物**。改了 `lot_layout.py` 里的车位几何
（位置 / 朝向 / 尺寸）却忘记重新生成 yaml，运行时就会继续用旧数据 ——
现象是「目标朝向不对」「预备位姿跑到人行道里」「入位报 2D 栅格不连通」，
极其难查（本项目实际踩过：车位朝向翻转后预备位姿被甩到 lot_ring_w 人行道上）。

本脚本做三类检查
----------------
[1] 车位集合一致：数量、id、类型
[2] 车位几何一致：cx/cy/yaw/l/w（带容差）
[3] 入位位姿可通行：每个车位的预备位姿（staging）必须落在可通行栅格上；
    顺带统计目标位姿被占用的车位（场内停着别的车，属正常，仅提示）

用法
----
    python3 check_layout.py            # 退出码 0 = 一致，1 = 不一致
    python3 check_layout.py --verbose  # 额外逐条打印每个车位
"""
import math
import os
import sys

sys.path.insert(0, os.path.expanduser("~/apa_ws/src/parking_planner"))

from parking_planner.lot_layout import Layout          # noqa: E402

WORLD = os.path.expanduser("~/apa_ws/src/parking_lot_city/worlds/city_parking_lot.sdf")
YAML_CANDIDATES = [
    os.path.expanduser("~/apa_ws/install/parking_planner/share/parking_planner/config/lot_layout.yaml"),
    os.path.expanduser("~/apa_ws/src/parking_planner/config/lot_layout.yaml"),
]

POS_TOL = 1e-3        # m
YAW_TOL = 1e-3        # rad
SIZE_TOL = 1e-3       # m


def find_yaml():
    for p in YAML_CANDIDATES:
        if os.path.isfile(p):
            return p
    return None


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    verbose = "--verbose" in sys.argv

    if not os.path.isfile(WORLD):
        print(f"[FATAL] 找不到 world：{WORLD}")
        return 2
    yaml_path = find_yaml()
    if not yaml_path:
        print("[FATAL] 找不到 lot_layout.yaml，先跑 make_layout")
        return 2

    print("=" * 66)
    print("车位布局一致性校验")
    print("=" * 66)
    print(f"  world : {WORLD}")
    print(f"  yaml  : {yaml_path}")

    fresh = Layout.from_world(WORLD)
    cached = Layout.load(yaml_path)

    bad = []

    # ---------------------------------------------------------------- [1] 集合
    fresh_ids = [s.id for s in fresh.stalls]
    cached_ids = [s.id for s in cached.stalls]
    print(f"\n[1] 车位集合：world {len(fresh_ids)} 个，yaml {len(cached_ids)} 个")
    if len(fresh_ids) != len(cached_ids):
        bad.append(f"数量不一致：world={len(fresh_ids)} yaml={len(cached_ids)}")
    missing = [i for i in fresh_ids if i not in cached_ids]
    extra = [i for i in cached_ids if i not in fresh_ids]
    if missing:
        bad.append(f"yaml 缺少车位：{missing[:10]}")
    if extra:
        bad.append(f"yaml 多出车位：{extra[:10]}")
    if not missing and not extra and len(fresh_ids) == len(cached_ids):
        print("    [PASS]")

    # ---------------------------------------------------------------- [2] 几何
    cached_map = {s.id: s for s in cached.stalls}
    print("\n[2] 车位几何（cx / cy / yaw / l / w）")
    n_geo_bad = 0
    for st in fresh.stalls:
        old = cached_map.get(st.id)
        if old is None:
            continue
        dpos = math.hypot(st.cx - old.cx, st.cy - old.cy)
        dyaw = abs(wrap(st.yaw - old.yaw))
        dsz = max(abs(st.l - old.l), abs(st.w - old.w))
        if dpos > POS_TOL or dyaw > YAW_TOL or dsz > SIZE_TOL:
            n_geo_bad += 1
            bad.append(
                f"{st.id} 几何不一致：Δpos={dpos:.3f} m "
                f"Δyaw={math.degrees(dyaw):.1f}° Δsize={dsz:.3f} m "
                f"（world yaw={math.degrees(st.yaw):.0f}° / "
                f"yaml yaw={math.degrees(old.yaw):.0f}°）")
        elif verbose:
            print(f"    {st.id:6s} OK  c=({st.cx:+.2f},{st.cy:+.2f}) "
                  f"yaw={math.degrees(st.yaw):+6.0f}° {st.kind}")
    if n_geo_bad == 0:
        print("    [PASS]")
    else:
        print(f"    [FAIL] {n_geo_bad} 个车位几何与 world 不一致")

    # ------------------------------------------------------ [3] 入位位姿可通行
    print("\n[3] 入位位姿可通行性（按 world 重建占用栅格）")
    grid = fresh.occupancy_grid(0.10)
    stg_bad, goal_busy = [], []
    for st in fresh.stalls:
        if not grid.free(st.staging_pose[0], st.staging_pose[1]):
            stg_bad.append(st.id)
        if not grid.free(st.goal_pose[0], st.goal_pose[1]):
            goal_busy.append(st.id)
    print(f"    目标位姿被占用 {len(goal_busy)} 个（场内停着别的车，属正常）")
    if stg_bad:
        print(f"    [FAIL] 预备位姿落在障碍里：{stg_bad}")
        print("           这些车位一旦被选中，入位规划必然失败。"
              "处理方式：排除该车位，或调整该排车位的 staging 偏移。")
    else:
        print(f"    [PASS] 全部 {len(fresh.stalls)} 个预备位姿均可通行")

    # ---------------------------------------------- [4] 垂直车位是否「一把倒车」
    print("\n[4] 垂直车位入位方式（预备位姿朝向应与车位一致 = 直线倒车）")
    n_perp = n_straight = 0
    for st in fresh.stalls:
        if st.kind != 'perpendicular':
            continue
        n_perp += 1
        d = abs(wrap(st.goal_pose[2] - st.staging_pose[2]))
        if d < math.radians(1.0):
            n_straight += 1
    print(f"    垂直车位 {n_perp} 个，其中 {n_straight} 个可从预备位姿直线倒车入位")
    if n_perp and n_straight != n_perp:
        print(f"    [WARN] 有 {n_perp - n_straight} 个垂直车位朝向不匹配，"
              f"无法一把倒车")

    # ---------------------------------------------------------------- 汇总
    print("\n" + "=" * 66)
    if bad:
        print("结果：yaml 与 world 不同步 [FAIL]")
        print("      先重新生成，再复查：")
        print("        cd ~/apa_ws && python3 -m parking_planner.make_layout")
        print()
        for b in bad:
            print(f"  · {b}")
        print("\n  注意：第 [3] 项的预备位姿冲突属于布局几何问题，"
              "重新生成 yaml 不会消除，\n"
              "        需要排除该车位或调整 staging 偏移。")
        return 1
    if stg_bad:
        print("结果：yaml 与 world 同步 [PASS]，但有车位不可用 [FAIL]")
        print(f"      预备位姿被占用的车位：{stg_bad}")
        return 1
    print("结果：一致，yaml 与 world 同步 [PASS]")
    return 0


if __name__ == "__main__":
    sys.exit(main())
