#!/bin/bash
# 轮询 / 取证：B 批真机结果 + B3 间隙来源定位
echo "===== 1. 批次进度（real_batch.csv）====="
iconv -f GBK -t UTF-8 /tmp/real_batch.csv 2>/dev/null | cut -c1-160 || cut -c1-160 /tmp/real_batch.csv

echo
echo "===== 2. B 相关日志文件 ====="
ls -la --time-style=+%H:%M /tmp/real_*.log 2>/dev/null | tail -10

echo
echo "===== 3. B3 单次日志：护栏 / 规划 / 验收 ====="
L=$(ls -t /tmp/real_B3_a*.log 2>/dev/null | head -1)
if [ -z "$L" ]; then
  L=$(ls -t /tmp/*B3*.log 2>/dev/null | head -1)
fi
echo "log=$L"
if [ -n "$L" ]; then
  echo "--- 行数 $(wc -l < "$L") ---"
  grep -nE '入位贴合护栏|车体最小间隙|泊车规划成功|位姿对照|评测|INCOMPLETE|DONE' "$L" | tail -20
  echo
  echo "--- 间隙 0.00 相关上下文 ---"
  grep -nE '0\.0[0-2] m' "$L" | tail -15
fi

echo
echo "===== 4. 进程 / 当前阶段 ====="
pgrep -af 'batch_real|gz sim|parking_planner' 2>/dev/null | cut -c1-120 | head -8
