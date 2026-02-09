#!/bin/bash

# 文件名建议：launch_can_bridge.sh
# 用法：source 这个脚本所在的环境（colcon build 后 source install/setup.bash），然后 ./launch_can_bridge.sh
# 功能：独立启动 CAN bridge（socketcan）节点，支持 can0 (经典 CAN) 和 can1 (CAN FD)
#       节点名唯一，避免冲突
#       启动后自动 configure + activate (lifecycle)


sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up
sudo ip link set can1 type can bitrate 1000000 dbitrate 5000000 fd on
sudo ip link set can1 up


echo "Starting CAN bridge nodes..."

# 启动 can1 (CAN FD) sender 和 receiver
ros2 run ros2_socketcan socket_can_sender_node_exe \
  --ros-args \
  -p interface:=can1 \
  -p enable_can_fd:=true \
  -r __node:=sender_can1 &

SENDER_CAN1_PID=$!

ros2 run ros2_socketcan socket_can_receiver_node_exe \
  --ros-args \
  -p interface:=can1 \
  -p enable_can_fd:=true \
  -r __node:=receiver_can1 &

RECEIVER_CAN1_PID=$!

# 启动 can0 (经典 CAN) sender 和 receiver
ros2 run ros2_socketcan socket_can_sender_node_exe \
  --ros-args \
  -p interface:=can0 \
  -p enable_can_fd:=false \
  -r __node:=sender_can0 &

SENDER_CAN0_PID=$!

ros2 run ros2_socketcan socket_can_receiver_node_exe \
  --ros-args \
  -p interface:=can0 \
  -p enable_can_fd:=false \
  -r __node:=receiver_can0 &

RECEIVER_CAN0_PID=$!

echo "All 4 socketcan nodes launched in background..."
echo "PIDs: sender_can1($SENDER_CAN1_PID) receiver_can1($RECEIVER_CAN1_PID) sender_can0($SENDER_CAN0_PID) receiver_can0($RECEIVER_CAN0_PID)"
echo "Waiting 5 seconds for nodes to fully initialize..."

sleep 2

echo "Configuring lifecycle nodes..."

ros2 lifecycle set /sender_can1 configure
ros2 lifecycle set /receiver_can1 configure
ros2 lifecycle set /sender_can0 configure
ros2 lifecycle set /receiver_can0 configure

echo "Activating lifecycle nodes..."

ros2 lifecycle set /sender_can1 activate
ros2 lifecycle set /receiver_can1 activate
ros2 lifecycle set /sender_can0 activate
ros2 lifecycle set /receiver_can0 activate


# 可选：前台等待所有节点（按 Ctrl+C 结束时一起关闭）
wait $SENDER_CAN1_PID $RECEIVER_CAN1_PID $SENDER_CAN0_PID $RECEIVER_CAN0_PID