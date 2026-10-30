#!/bin/bash

# 项目根目录（由本脚本位置推出，克隆到任意路径都可用）
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
# 重新生成 lot_layout.yaml（修复：17:11 更新 world 后布局未同步）
# 然后离线闭环验证 B 排空位 B1/B3/B5
WS="$HOME/apa_ws"
W="$WS/src/parking_lot_city/worlds/city_parking_lot.sdf"
L="$WS/src/parking_planner/config/lot_layout.yaml"

for d in /opt/ros/*/setup.bash; do source "$d"; break; done
source "$WS/install/setup.bash"

echo "===== A. world 实际几何（sidewalk_south_road / parked_b / back_stall_edge）====="
grep -A2 'model name="sidewalk_south_road' "$W" | grep -E 'model name|<pose>' | sed 's/^ *//'
echo '---'
grep -o '<name>parked_b[0-9]*</name><pose>[^<]*' "$W"
echo '---'
grep -A2 'model name="back_stall_edge' "$W" | grep -E 'model name|<pose>|<size>' | sed 's/^ *//'

echo
echo "===== B. 重新生成 lot_layout.yaml ====="
ros2 run parking_planner make_layout --world "$W" --out "$L" || { echo "make_layout 失败"; exit 1; }

echo
echo "===== C. 新布局里的 B 排车位与停放车 ====="
grep -n "parked_b\|name: B[0-9]" "$L"

echo
echo "===== D. 离线闭环验证 B1 B3 B5 ====="
VERIFY=""
for c in "$WS/scripts/verify_park_sim.py" "$WS/src/scripts/verify_park_sim.py" \
         "$WS/src/parking_planner/scripts/verify_park_sim.py"; do
  [ -f "$c" ] && VERIFY="$c" && break
done
[ -z "$VERIFY" ] && VERIFY="$ROOT/scripts/verify_park_sim.py"
echo "使用脚本: $VERIFY"
python3 "$VERIFY" B1 B3 B5 > /tmp/verify_b135.log 2>&1
rc=$?
echo "退出码: $rc  完整日志: /tmp/verify_b135.log"
echo "--- 摘要（每车位结论行）---"
grep -E 'B[0-9]|间隙|余量|误差|剐|不达标|达标|失败|成功' /tmp/verify_b135.log | tail -50
