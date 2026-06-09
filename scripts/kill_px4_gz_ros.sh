#!/usr/bin/env bash

echo "======================================"
echo "[CLEAN] Kill PX4 + Gazebo/GZ + ROS processes"
echo "======================================"

USER_NAME="$(whoami)"

# -------------------------------
# 1. 要清理的进程关键字
# -------------------------------
PATTERNS=(
    "px4"
    "gz sim"
    "gzserver"
    "gzclient"
    "gazebo"
    "ign gazebo"
    "ignition"
    "roslaunch"
    "rosmaster"
    "roscore"
    "rosout"
    "rviz"
    "mavros"
    "mavros_node"
    "ros_gz_bridge"
    "parameter_bridge"
    "static_transform_publisher"
    "robot_state_publisher"
    "joint_state_publisher"
    "MicroXRCEAgent"
    "micrortps_agent"
    "mavlink-router"
    "MAVProxy"
)

echo ""
echo "[1/5] Send SIGINT..."
for p in "${PATTERNS[@]}"; do
    pkill -2 -u "$USER_NAME" -f "$p" 2>/dev/null
done

sleep 1

echo "[2/5] Send SIGTERM..."
for p in "${PATTERNS[@]}"; do
    pkill -15 -u "$USER_NAME" -f "$p" 2>/dev/null
done

sleep 1

echo "[3/5] Send SIGKILL to remaining processes..."
for p in "${PATTERNS[@]}"; do
    pkill -9 -u "$USER_NAME" -f "$p" 2>/dev/null
done

# -------------------------------
# 2. 清理常见端口占用
# -------------------------------
echo "[4/5] Free common ROS / PX4 / Gazebo ports..."

PORTS=(
    11311   # ROS master
    14540   # PX4 MAVLink common
    14550   # QGC MAVLink common
    14557   # PX4 MAVLink common
    14580
    18570
    4560    # PX4 simulator TCP/UDP common
)

for port in "${PORTS[@]}"; do
    if command -v fuser >/dev/null 2>&1; then
        fuser -k "${port}/tcp" 2>/dev/null
        fuser -k "${port}/udp" 2>/dev/null
    fi
done

# -------------------------------
# 3. 清理 Gazebo / GZ / ROS 临时文件
# -------------------------------
echo "[5/5] Clean temporary files..."

rm -rf /tmp/gazebo-* 2>/dev/null
rm -rf /tmp/ignition-* 2>/dev/null
rm -rf /tmp/gz-* 2>/dev/null
rm -rf /tmp/roslaunch-* 2>/dev/null
rm -rf /tmp/.gazebo-* 2>/dev/null

# -------------------------------
# 4. 输出残留检查
# -------------------------------
echo ""
echo "======================================"
echo "[CHECK] Remaining related processes:"
echo "======================================"

ps aux | grep -E "px4|gz sim|gzserver|gzclient|gazebo|roslaunch|rosmaster|roscore|rosout|rviz|mavros|ros_gz_bridge|parameter_bridge|MicroXRCEAgent|mavlink-router|MAVProxy" | grep -v grep

echo ""
echo "======================================"
echo "[DONE] PX4 + GZ/Gazebo + ROS cleanup finished."
echo "======================================"