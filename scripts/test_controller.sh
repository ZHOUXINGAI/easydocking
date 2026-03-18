#!/bin/bash
# EasyDocking 测试脚本

echo "========================================"
echo "   EasyDocking Test Script"
echo "========================================"
echo ""

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT_DIR/scripts/setup_local_env.sh"

# 加载工作空间
if [ -d "$ROOT_DIR/install" ]; then
    source "$ROOT_DIR/install/setup.bash"
else
    echo "Workspace not built. Run: colcon build"
    exit 1
fi

echo "Testing Docking Controller..."

# 启动控制器节点
echo "Starting docking controller..."
ros2 run easydocking_control docking_controller_node &
CONTROLLER_PID=$!
sleep 2

# 测试命令发布
echo "Publishing START command..."
ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'START'}"

echo "Waiting 5 seconds..."
sleep 5

# 检查状态
echo "Checking docking status..."
timeout 5 ros2 topic echo /docking/status &
ECHO_PID=$!
sleep 2
kill $ECHO_PID 2>/dev/null

# 停止控制器
echo "Stopping controller..."
ros2 topic pub --once /docking/command easydocking_msgs/msg/DockingCommand "{command: 'STOP'}"
sleep 1

kill $CONTROLLER_PID 2>/dev/null

echo "Test completed!"
