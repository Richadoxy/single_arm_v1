#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import JointState
import numpy as np
import sys
import csv
import time
from collections import deque

class TrajectoryPlayer(Node):
    def __init__(self):
        super().__init__('trajectory_player')
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']
        
        # 新增：关节名 → 索引 的映射（在收到第一个有效 joint_states 后建立）
        self.joint_index_map = {}          # dict: str → int
        self.map_initialized = False       # 是否已建立映射
        # Action client
        self.action_client = ActionClient(
            self, 
            FollowJointTrajectory, 
            '/single_arm_controller/follow_joint_trajectory'
        )
        
        
        # 订阅 joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # 数据存储（使用 list of lists 或 deque，根据需要选择）
        self.save_time_linux = []
        self.save_q = []        # actual position
        self.save_dq = []       # actual velocity
        self.save_tau = []      # actual effort (电流/力矩)
        
        # 参考轨迹（从文件读取）
        self.ref_trajectory = None  # 将存储 (N, 19) 的 numpy 数组
        
        # 轨迹是否已发送
        self.trajectory_sent = False
        self.goal_handle = None

    def load_trajectory(self, file_path):
        data = np.loadtxt(file_path, delimiter=',')
        if data.shape[0] != 19:
            self.get_logger().error('文件格式错误：预期19行')
            sys.exit(1)
        data = data.T  # (N, 19)
        
        self.ref_trajectory = data
        self.get_logger().info(f'已加载参考轨迹：{data.shape[0]} 个点')
        
        # 构建 JointTrajectory 消息
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names
        
        for i in range(len(data)):
            point = JointTrajectoryPoint()
            point.positions = data[i, 0:6].tolist()
            point.velocities = data[i, 6:12].tolist()
            point.accelerations = data[i, 12:18].tolist()
            t_sec = int(data[i, 18])
            t_nsec = int((data[i, 18] - t_sec) * 1e9)
            point.time_from_start = Duration(sec=t_sec, nanosec=t_nsec)
            trajectory.points.append(point)
        
        return trajectory

    def joint_state_callback(self, msg: JointState):
        if not self.trajectory_sent:
            return
        
        # 第一次收到消息时，建立关节索引映射
        if not self.map_initialized:
            self.joint_index_map = {}
            missing_joints = []
            
            for joint in self.joint_names:
                try:
                    idx = msg.name.index(joint)
                    self.joint_index_map[joint] = idx
                except ValueError:
                    missing_joints.append(joint)
            
            if missing_joints:
                self.get_logger().warn(
                    f"joint_states 中缺少关节: {', '.join(missing_joints)}，后续记录可能不完整"
                )
            else:
                self.get_logger().info("joint_states 关节索引映射已建立")
            
            self.map_initialized = True
        
        # 如果映射还没建立，或者消息中完全没有我们需要的关节，就跳过
        if not self.joint_index_map:
            return
        
        current_time = time.time()
        
        q_actual = []
        dq_actual = []
        tau_actual = []
        
        # 按关节名顺序取值（保证顺序永远是 joint1~joint6）
        for joint in self.joint_names:
            if joint in self.joint_index_map:
                idx = self.joint_index_map[joint]
                q_actual.append(msg.position[idx] if idx < len(msg.position) else 0.0)
                dq_actual.append(msg.velocity[idx] if idx < len(msg.velocity) else 0.0)
                tau_actual.append(msg.effort[idx] if idx < len(msg.effort) else 0.0)
            else:
                # 如果这个关节在当前消息中缺失，用 0 或上一次值（这里用 0）
                q_actual.append(0.0)
                dq_actual.append(0.0)
                tau_actual.append(0.0)
                self.get_logger().warn_once(f"关节 {joint} 在本次 joint_states 中缺失，使用默认值 0")

        self.save_time_linux.append(current_time)
        self.save_q.append(q_actual)
        self.save_dq.append(dq_actual)
        self.save_tau.append(tau_actual)

    def send_trajectory(self, trajectory):
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('Action server not available!')
            return

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = trajectory

        self.get_logger().info('Sending trajectory goal...')
        future = self.action_client.send_goal_async(goal_msg)
        future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return
        self.get_logger().info('Goal accepted')
        self.trajectory_sent = True  # 开始记录数据
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info('Trajectory executed successfully')
        else:
            self.get_logger().error(f'Trajectory failed with error code: {result.error_code}')
        
        # 轨迹执行完后保存数据
        self.save_to_file()
        rclpy.shutdown()

    def save_to_file(self):
        if not self.ref_trajectory.any():
            self.get_logger().error('参考轨迹为空，无法保存')
            return
        
        filename = f"recorded_trajectory_{time.strftime('%Y%m%d_%H%M%S')}.txt"
        
        try:
            with open(filename, 'w', newline='') as file:
                writer = csv.writer(file, delimiter=',')
                
                # 记录的点数可能和参考轨迹不同，取较小的长度
                record_len = len(self.save_time_linux)
                ref_len = self.ref_trajectory.shape[0]
                min_len = min(record_len, ref_len)
                
                self.get_logger().info(f'保存 {min_len} 个数据点（实际记录 {record_len}，参考 {ref_len}）')
                
                for cnt in range(min_len):
                    row = [f"{self.save_time_linux[cnt]:.6f}"]
                    row.extend([f"{x:.6f}" for x in self.save_q[cnt]])
                    row.extend([f"{x:.6f}" for x in self.save_dq[cnt]])
                    row.extend([f"{x:.6f}" for x in self.save_tau[cnt]])
                    
                    # 参考值
                    row.extend([f"{x:.6f}" for x in self.ref_trajectory[cnt, 0:6]])   # qr
                    row.extend([f"{x:.6f}" for x in self.ref_trajectory[cnt, 6:12]])  # dqr
                    row.extend([f"{x:.6f}" for x in self.ref_trajectory[cnt, 12:18]]) # ddqr
                    
                    writer.writerow(row)
            
            self.get_logger().info(f'数据已保存到 {filename}，总点数: {min_len}')
        
        except Exception as e:
            self.get_logger().error(f'保存文件失败: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryPlayer()
    
    # 替换成你的文件路径
    file_path = '/home/xyd/single_arm_ros2/src/single_arm_control/scripts/singleArm_data_read_excitation_134.5.txt'
    trajectory = node.load_trajectory(file_path)
    node.send_trajectory(trajectory)
    
    rclpy.spin(node)

if __name__ == '__main__':
    main()