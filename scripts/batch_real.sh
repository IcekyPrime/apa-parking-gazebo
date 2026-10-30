#!/bin/bash
# 逐车位真机回归（headless Gazebo，与 run_gui.sh 同一套场景与逻辑）。
#
#   LIST="W1 W4 W5" bash scripts/batch_real.sh          # 只跑指定车位
#   bash scripts/batch_real.sh                          # 跑默认的 11 个垂直空位
#   LIST="P1 P3 B1" TMO=900 bash scripts/batch_real.sh  # 平行位，超时放宽
#
# 每个车位：硬清场 → 启仿真 → **等评测行出现就收工**（不等 460 s 超时）
# → 硬清场，结果追加到 $OUT。
WS=${WS:-$HOME/apa_ws}
# 以 root 运行时 $HOME 会变成 /root，工作区就不在默认位置了；这里兜底一次，
# 允许用 WS=/path/to/ws 显式指定。
[ -f "$WS/install/setup.bash" ] || WS=$HOME/apa_ws
OUT=${OUT:-/tmp/real_batch.csv}
LIST=${LIST:-"W1 W4 W5 W7 W9 E0 E2 E3 E6 E7 E8"}
TMO=${TMO:-540}
# 主路干扰车 / 行人：**必须保持 true**。
# 关掉 agent 并不会让车上/行人消失 —— 它们是 world 文件里 `<include>` 出来的
# 实体，agent 只是驱动它们移动；关掉之后它们就**原地不动地站在场景里**，
# 反而变成路障（实测：W7 一轮在离场 8.5 m 处被一个静止实体顶住 117 s，
# 命中点恒为 lon=2.97 lat=+0.68）。
TRAFFIC=${TRAFFIC:-true}
PEDS=${PEDS:-true}
# 批量回归时把出生点挪到主干道中段（-45 → -30）：路线从 71 m 缩到 ~62 m，
# 但**不能太靠西**——`SX=-5` 会把车直接生在行人 `pedestrian_north` 的航线上，
# 起步 1.5 s 就被行人顶住，然后行人「退 2 m → 等 10.5 s → 再前进」和自车的
# 4.5 m 护栏互相锁死，连续 3 次尝试都在 37 m 之外原地不动。
SX=${SX:--30.0}
SY=${SY:-15.65}
SYAW=${SYAW:-0.0}
# 位姿真值对照（/vehicle_pose vs /odom_gt）。默认开：每次尝试单独一份
# `/tmp/pose_<车位>_a<n>.csv` + `_summary.txt`，并往 $OUT 里写一行摘要。
# 关掉可省 ~3% CPU：AUDIT=0 bash scripts/batch_real.sh
AUDIT=${AUDIT:-1}
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export LIBGL_ALWAYS_SOFTWARE=1
export DISPLAY=${DISPLAY:-:0}
export PYTHONUNBUFFERED=1
export FASTRTPS_DEFAULT_PROFILES_FILE=\
"$WS/install/parking_planner/share/parking_planner/config/fastdds_udp_only.xml"

: > "$OUT"
echo "车位 | 泊车结果 | 评测行（终态/误差/风险）" | tee -a "$OUT"

for g in $LIST; do
  attempt=1
  while [ "$attempt" -le 3 ]; do
  bash "$HERE/hard_clean.sh" >/dev/null 2>&1
  # [本轮修复] 起飞前断言：**最多只能有一个 gz 服务端**。
  # 孤儿 gz（ppid=1）会让两个 Gazebo 同跑，8 GB 内存直接打爆，之后所有节点
  # exit -9 —— 真机 P1 的两次「路线超时」其实是 OOM，被误判成算法问题。
  # 注意 `pgrep -c` 无匹配时返回 1，不能写 `$(pgrep -c ... || echo 0)`
  # （那会拼成 "0\n0"），统一用下面的 gz_count 函数。
  gz_count() { local n; n=$(pgrep -cf 'gz sim' 2>/dev/null); echo "${n:-0}"; }
  ngz=$(gz_count)
  if [ "$ngz" -gt 0 ]; then
    echo "$g | 起飞前仍有 $ngz 个残留 gz，再清一次" | tee -a "$OUT"
    sleep 8
    bash "$HERE/hard_clean.sh" >/dev/null 2>&1
    ngz=$(gz_count)
    if [ "$ngz" -gt 0 ]; then
      echo "$g | 清理失败（$ngz 个 gz 仍在），跳过本车位" | tee -a "$OUT"
      break
    fi
  fi
  # 每次尝试单独一个日志文件：重试时不要把上一次的诊断信息覆盖掉
  LOG=/tmp/real_${g}_a${attempt}.log
  # [同学 C 修复] 必须先 rm 再写。曾经出现过「用 root 跑过一次，/tmp/real_*.log
  # 变成 root 所有」的情况：之后以普通用户跑时 `: > "$LOG"` 静默失败，脚本
  # 直接 grep 到**上一轮的旧日志**，于是 3 个车位在 40 秒内「跑完」，
  # 输出了和上一轮一字不差的结果（W4 2.84 m / W5 3.80 m）——差点被当成新结论。
  rm -f "$LOG" 2>/dev/null
  if ! : > "$LOG" 2>/dev/null; then
    echo "$g | 无法写日志 $LOG（残留文件属主不对？sudo rm -f $LOG）" | tee -a "$OUT"
    break
  fi
  # ---- 位姿真值对照：在 launch 之前起，随 launch 的 SIGINT 一起收尾 ----
  PCSV=/tmp/pose_${g}_a${attempt}.csv
  APID=""
  if [ "$AUDIT" = "1" ]; then
    rm -f "$PCSV" 2>/dev/null
    python3 "$HERE/pose_audit.py" --csv "$PCSV" >/dev/null 2>&1 &
    APID=$!
  fi
  # [本轮修复] setsid + exec 启动，解决「批量永久卡死」：
  # 旧写法 `( ros2 launch ... ) &` 在非交互 shell 里，后台作业会忽略 SIGINT，
  # 超时后 `kill -INT` 打不动它，紧接着的 `wait "$PID"` 就永久挂起 ——
  # 实测 P1 在 900 s 判据结束后又挂了 20 多分钟没往下一个车位走。
  # 改成 setsid 自成进程组 + exec 让 PID 就是 ros2 launch 本身，
  # 收尾时先 SIGINT 优雅退，25 s 不退就整组 -9。
  setsid bash -c "exec ros2 launch parking_planner parking_demo.launch.py \
      headless:=true use_rviz:=false use_prediction:=true \
      drive_patrol:=false use_parker:=false use_traffic:=$TRAFFIC \
      use_peds:=$PEDS \
      spawn_x:=$SX spawn_y:=$SY spawn_yaw:=$SYAW \
      goal_stall:=$g" > "$LOG" 2>&1 &
  PID=$!
  for _ in $(seq 1 "$TMO"); do
    sleep 1
    # 结束判据：**泊车结果出来就算结束**，不再强求评测节点也打 【评测】 行。
    # [本轮修复] 旧判据要求两条都在，而停车成功后 parking_eval 有时不打评测行，
    # 于是这一轮虽然已经 `泊车完成：终点误差 0.09 m` 仍被判成「被中断」整轮重跑
    # —— 真机 SW1/SE1 都白跑了一次（各 150~240 s）。评测行缺失改为只记一行说明。
    if grep -qE '泊车完成|泊车未达标' "$LOG"; then
      # 再给评测节点 12 s 把汇总行打出来，然后收工
      for _ in $(seq 1 12); do
        grep -qE '\[评测\]' "$LOG" && break
        sleep 1
      done
      break
    fi
    if grep -qE '全局路线行驶超时，放弃|规划超时，放弃' "$LOG"; then sleep 8; break; fi
    if ! kill -0 "$PID" 2>/dev/null; then break; fi
  done
  sleep 2
  kill -INT "$PID" 2>/dev/null
  # 优雅退出最多等 25 s；还不退就整组强杀（setsid 保证 PID 即进程组号）
  for _ in $(seq 1 25); do
    kill -0 "$PID" 2>/dev/null || break
    sleep 1
  done
  if kill -0 "$PID" 2>/dev/null; then
    kill -9 -- "-$PID" 2>/dev/null
    kill -9 "$PID" 2>/dev/null
  fi
  wait "$PID" 2>/dev/null
  # 采集节点收尾（SIGINT 会让它 flush + 打印帧数；不响应就硬杀，数据已在缓冲里）
  if [ -n "$APID" ]; then
    kill -INT "$APID" 2>/dev/null
    for _ in $(seq 1 20); do
      kill -0 "$APID" 2>/dev/null || break
      sleep 0.25
    done
    kill -9 "$APID" 2>/dev/null
    wait "$APID" 2>/dev/null
  fi
  line=$(grep -E '\[评测\]' "$LOG" | tail -1 | sed -E 's/.*\[评测\] //')
  done_line=$(grep -E '泊车完成|泊车未达标|放弃' "$LOG" | tail -1 \
              | sed -E 's/.*parking_planner\]: //')
  [ -z "$line" ] && line="（未产生评测行，见 $LOG）"
  # ---- 位姿对照摘要：滞后 / 残差 / 航向 ----
  if [ -n "$APID" ] && [ -s "$PCSV" ]; then
    PS="${PCSV%.csv}_summary.txt"
    python3 "$HERE/pose_audit.py" --analyze "$PCSV" > "$PS" 2>/dev/null
    pose_line=$(grep -E '最佳滞后|在 τ\*|个估计' "$PS" | tr '\n' ' ')
    echo "$g | 位姿对照 | $pose_line（详见 $PS）" | tee -a "$OUT"
  fi
  # 只有「路线超时放弃」才重跑（最多 3 次）；只要出过泊车结果就记账
  # [本轮修复] 旧判据是「没有 [评测] 行就重跑」，会把已经成功的一轮也重跑。
  if ! grep -qE '泊车完成|泊车未达标' "$LOG" && [ "$attempt" -lt 3 ]; then
    attempt=$((attempt + 1))
    echo "$g | 第 $((attempt - 1)) 次被中断（路线超时/放弃），重试" | tee -a "$OUT"
    continue
  fi
  echo "$g | $done_line | $line" | tee -a "$OUT"
  break
  done
done
echo
echo "==== 汇总（$OUT）===="
cat "$OUT"
