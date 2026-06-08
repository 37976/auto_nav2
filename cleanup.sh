#!/bin/bash
# -*- coding: utf-8 -*-
# cleanup.sh — 在启动 Gazebo+Nav 之前清理上一次运行残留，避免第二次启动卡顿。
#
# 用法:
#   bash cleanup.sh          自动模式（仅清理文件和端口，从 launch 文件调用）
#   bash cleanup.sh --kill   手动模式（额外终止残留进程）

set -euo pipefail

DO_KILL=false
if [[ "${1:-}" == "--kill" ]]; then
  DO_KILL=true
fi

echo "=========================================="
echo "  auto_nav2 启动前清理"
echo "  模式: $($DO_KILL && echo '手动(含进程终止)' || echo '自动(仅资源清理)')"
echo "=========================================="

# ---------------------------------------------------------------------------
# 1. (仅手动模式) 终止可能残留的 Gazebo / ROS 2 进程
# ---------------------------------------------------------------------------
if $DO_KILL; then
  echo "[1/4] 终止残留进程..."

  # 二进制进程：精确匹配可执行文件名
  kill -0 $$ 2>/dev/null  # no-op, self-check
  for bin in gzserver gzclient rviz2; do
    if pkill -x "$bin" 2>/dev/null; then
      echo "  已终止: $bin"
    fi
  done

  # Python 可执行名（用 ros2 node 对应的 executable 名，匹配命令行末尾）
  for exe in robot_state_publisher xfeat_rgbd_odometry voronoi_node \
             static_map_server lidar_global_localize start_nav \
             odom_tf_bridge odom_to_map_relay map_once_relay \
             map_pub odom_map_tf points_pub_map pose_logger \
             laser_scan_to_points odom_fusion_node; do
    # 只匹配以该可执行名结尾的进程，避免误杀 launch 文件
    if pkill -f "[${exe:0:1}]${exe:1}" 2>/dev/null; then
      echo "  已终止: $exe"
    fi
  done

  # 等待进程完全退出
  sleep 1
  echo "  进程终止完成"
else
  echo "[1/3] 跳过进程终止（自动模式）"
fi

# ---------------------------------------------------------------------------
# 2. 清理 FastDDS (rmw_fastrtps_cpp) 共享内存僵尸文件
# ---------------------------------------------------------------------------
STEP_NUM=$($DO_KILL && echo "2" || echo "2")
echo "[$STEP_NUM/4] 清理 FastDDS 共享内存僵尸文件..."

if command -v fastdds &>/dev/null; then
  fastdds shm clean 2>&1 || true
  echo "  fastdds shm clean 完成"
else
  SHM_DIR="/dev/shm"
  CLEANED=0
  for f in "$SHM_DIR"/fastrtps_* "$SHM_DIR"/fastrtps_port*; do
    if [ -f "$f" ]; then
      if flock -n "$f" -c "true" 2>/dev/null; then
        rm -f "$f"
        CLEANED=$((CLEANED + 1))
      fi
    fi
  done
  echo "  手动清理了 $CLEANED 个僵尸文件"
fi

# ---------------------------------------------------------------------------
# 3. 等待 Gazebo 端口 11345 释放
# ---------------------------------------------------------------------------
STEP_NUM=$($DO_KILL && echo "3" || echo "3")
echo "[$STEP_NUM/4] 等待 Gazebo 端口 11345 释放..."

MAX_WAIT_SEC=65
WAITED=0
while true; do
  if ss -tlnp 2>/dev/null | grep -q ":11345 "; then
    if [ "$WAITED" -ge "$MAX_WAIT_SEC" ]; then
      echo "  警告: 端口 11345 在 ${MAX_WAIT_SEC}s 内未释放，强制继续"
      break
    fi
    echo -n "."
    sleep 1
    WAITED=$((WAITED + 1))
  else
    echo ""
    echo "  端口 11345 已释放 (等待了 ${WAITED}s)"
    break
  fi
done

# ---------------------------------------------------------------------------
# 4. 清理 Gazebo 锁目录中的过期日志
# ---------------------------------------------------------------------------
STEP_NUM=$($DO_KILL && echo "4" || echo "4")
echo "[$STEP_NUM/4] 清理 Gazebo 残留日志..."
GAZEBO_DIR="$HOME/.gazebo"
if [ -d "$GAZEBO_DIR" ]; then
  for logdir in "$GAZEBO_DIR"/client-* "$GAZEBO_DIR"/server-*; do
    if [ -d "$logdir" ]; then
      rm -f "$logdir"/*.log 2>/dev/null || true
    fi
  done
  rm -f "$GAZEBO_DIR"/ogre.log 2>/dev/null || true
  echo "  Gazebo 日志已清理"
else
  echo "  无需清理"
fi

echo ""
echo "=========================================="
echo "  清理完成，可以启动项目了"
echo "=========================================="
