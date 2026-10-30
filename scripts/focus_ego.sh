#!/bin/bash
# ============================================================
# 把 Gazebo 3D 视角锁定到被操控小车（默认 ego_vehicle）
#
# 为什么需要它：gz sim 默认 GUI 相机固定在 `<camera_pose>-6 0 6 0 0.5 0</camera_pose>`，
# 也就是世界原点上方 6 m 的近景；而自车默认出生在主路 (-45, 15.65)，离原点 45 m，
# 一开窗口根本不在画面里 —— 这就是「在可视化界面里找不到操控小车」的直接原因。
# 实测：自车在 (-32.2, 15.7) 时，GUI 相机仍在 (-6, 0, 6)，相距 26 m，不在视野内。
#
# 用法：
#   bash scripts/focus_ego.sh              # 锁定 ego_vehicle
#   bash scripts/focus_ego.sh traffic_car  # 锁定某台干扰车
#   OX=-16 OZ=9 bash scripts/focus_ego.sh  # 自定义追车偏移（目标机体系：-x 在后、+z 在上）
#
# 实现：gz-gui 的 CameraTracking 插件（默认 gui.config 已加载）监听
#   /gui/track   gz.msgs.CameraTrack
# 发 track_mode=FOLLOW + follow_target + follow_offset，相机就会跟住该实体。
# 注意 1：本版本没有 /gui/follow 话题（旧版接口），只能走 /gui/track。
# 注意 2：gz topic 的参数顺序是  -t <话题> -m <消息类型> -p <数据>
#         （-t 是话题、-m 是类型，和直觉相反；写反了就报 "Unable to create message"）。
# ============================================================
ENTITY=${1:-ego_vehicle}
OX=${OX:--9.5}
OY=${OY:-0.0}
OZ=${OZ:-5.5}
PG=${PG:-0.45}
TRY=${TRY:-20}

if ! command -v gz >/dev/null 2>&1; then
  source /opt/ros/jazzy/setup.bash
fi

# 等 GUI 起来、CameraTracking 把 /gui/track 订阅上
for _ in $(seq 1 "$TRY"); do
  gz topic -i -t /gui/track 2>/dev/null | grep -q 'Subscribers' && break
  sleep 0.5
done

MSG='track_mode: FOLLOW,
follow_target: {name: "'"$ENTITY"'"},
follow_offset: {x: '"$OX"', y: '"$OY"', z: '"$OZ"'},
follow_pgain: '"$PG"''

# 用「相机到自车的水平距离」判断是否真的跟上了：
# 默认原点视角下相机固定在 (-6, 0)，离自车 25 m 以上；
# 追车视角下应稳定在 sqrt(OX^2+OY^2) 附近。
ok=0
for _ in $(seq 1 8); do
  gz topic -t /gui/track -m gz.msgs.CameraTrack -p "$MSG" >/dev/null 2>&1
  sleep 1.0
  cam=$(timeout 2 gz topic -e -t /gui/camera/pose -n 1 2>/dev/null | tr '\n' ' ')
  ego=$(gz model -m "$ENTITY" --pose 2>/dev/null | grep -oE '[-]?[0-9]+\.[0-9]+' | tr '\n' ' ')
  dist=$(/usr/bin/python3 -c '
import re, sys, math
cam, ego = sys.argv[1], sys.argv[2]
m = re.search(r"position\s*\{([^}]*)\}", cam)
if not m:
    print(-1); raise SystemExit
d = {k: float(v) for k, v in re.findall(r"(\w+)\s*:\s*(-?[\d.eE+-]+)", m.group(1))}
nums = [float(v) for v in ego.split()]
if len(nums) < 2:
    print(-1); raise SystemExit
print(round(math.hypot(d.get("x", 0.0) - nums[0], d.get("y", 0.0) - nums[1]), 2))
' "$cam" "$ego" 2>/dev/null)
  if [ -z "$dist" ]; then continue; fi
  if awk -v d="$dist" 'BEGIN{exit !(d > 4.0 && d < 20.0)}'; then
    ok=1
    echo "[focus_ego] 视角已切到追车模式，跟随 $ENTITY（偏移 $OX $OY $OZ）"
    echo "[focus_ego] 相机到自车水平距离 ${dist} m（默认原点视角下是 20 m 以上）"
    break
  fi
done

if [ "$ok" = "0" ]; then
  echo "[focus_ego] 未能确认已跟随 $ENTITY。"
  echo "[focus_ego] 若 Gazebo 窗口还没开、或实体尚未生成，稍后重跑本脚本即可。"
  echo "[focus_ego] 也可以手动在窗口里滚轮缩小、中键平移去找它。"
  exit 1
fi
