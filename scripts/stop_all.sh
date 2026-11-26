#!/usr/bin/env bash
# 安全结束本项目的仿真/建图/导航/RViz 进程（按 PID 精确处理，不误杀终端）
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pat='nav_stack/lib/nav_stack/(simulator|fastlio_map|bridge|localizer)|fastlio_mapping|nav2_(controller_server|planner_server|bt_navigator|behavior_server|smoother_server|waypoint_follower|velocity_smoother|lifecycle_manager)|ros2 launch nav_stack|robot_state_publisher.*__node:=(omni|br)_state_publisher|rviz2.*(mujoco_backend|__node:=(mapping|navigation)_rviz)'

collect() {
  local all
  all=$(ps -eo pid,args | grep -E "$pat" | grep -v grep | grep -v stop_all | awk '{print $1}')
  # 兼容旧版本留下的匿名 robot_state_publisher / RViz：用“工作目录属于本项目”识别
  for pid in $(ps -eo pid,args | grep -E 'robot_state_publisher|rviz2' | grep -v grep | awk '{print $1}'); do
    [ "$(readlink -f /proc/$pid/cwd 2>/dev/null)" = "$root" ] && all="$all $pid"
  done
  echo $all | tr ' ' '\n' | sort -u | tr '\n' ' '
}

pids=$(echo $(collect))
if [ -z "$pids" ]; then
  echo "没有发现本项目相关进程。"
  exit 0
fi
echo "结束进程: $pids"
kill -TERM $pids 2>/dev/null
sleep 3
left=$(echo $(collect))
[ -n "$left" ] && { echo "强制结束: $left"; kill -KILL $left 2>/dev/null; }
sleep 1
rm -f /dev/shm/fastrtps_* /dev/shm/fastdds_* 2>/dev/null
echo "已清理。"
