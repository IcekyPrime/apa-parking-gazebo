#!/bin/bash
# 等某份真机回归日志出现结果行（评测/完成/放弃），再把关键行打出来。
# 用法：bash scripts/watch_result.sh /tmp/real_E2_a1.log [最多等多少秒，默认280]
LOG=$1
MAXS=${2:-280}
[ -f "$LOG" ] || { echo "no log: $LOG"; exit 1; }
i=0
while [ $i -lt $((MAXS / 5)) ]; do
  grep -qE '\[评测\]|泊车完成|泊车未达标|全局路线行驶超时' "$LOG" 2>/dev/null && break
  sleep 5
  i=$((i + 1))
done
grep -E '僵持|倒退|泊车完成|泊车未达标|放弃|\[评测\]' "$LOG" 2>/dev/null | tail -15
