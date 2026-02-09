#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

import numpy as np

from grav_comp_controller import GravityCompensator

class GravityCompensationNode(Node):
    def __init__(self):
        super().__init__('gravity_compensation_node')

        # 声明并获取 robot_description 参数
        self.declare_parameter('robot_description', '')  # 默认空字符串
        urdf_string = self.get_parameter('robot_description').get_parameter_value().string_value

        if not urdf_string:
            self.get_logger().fatal("robot_description 参数为空，无法初始化！")
            raise ValueError("robot_description 参数为空")

        self.get_logger().info(f"从参数获取 robot_description，长度: {len(urdf_string)} 字符")

        # 直接写死配置（按你的需求）
        self.use_linearized = True          # True 用线性化模型，False 用 pinocchio 纯重力
        self.grav_gain = np.array([0.4, 0.45, 0.4, 1.0, 1.0, 1.0])
        self.fric_gain = np.array([0.30, 0.3, 0.4, 0.0, 0.0, 0.0])  # 如果不需要摩擦补偿就设 0

        self.joint_names = [
            'joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'
        ]

        self.compensator = None

        try:
            self.compensator = GravityCompensator(urdf_string)
            self.get_logger().info(f"Pinocchio 模型加载成功，自由度数: {self.compensator.nq}")

            # 直接订阅 joint_states（无需等待 URDF）
            self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_callback,
                10
            )

        except Exception as e:
            self.get_logger().fatal(f"初始化失败: {str(e)}")
            raise

        # 发布器（不变）
        self.pub_effort = self.create_publisher(
            Float64MultiArray,
            '/single_arm_effort_controller/commands',
            10
        )

    def robot_description_callback(self, msg: String):
        if self.compensator is not None:
            return  # 只处理一次

        urdf_string = msg.data
        self.get_logger().info(f"收到 robot_description，长度: {len(urdf_string)} 字符")

        try:
            self.compensator = GravityCompensator(urdf_string)
            self.get_logger().info(f"Pinocchio 模型加载成功，自由度数: {self.compensator.nq}")

            # 收到 URDF 后才订阅 joint_states
            self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_callback,
                10
            )

        except Exception as e:
            self.get_logger().fatal(f"初始化失败: {str(e)}")
            raise

    def joint_callback(self, msg: JointState):
        if self.compensator is None:
            return

        if not msg.position:
            return

        q = np.zeros(6, dtype=float)
        q_dot = np.zeros(6, dtype=float)

        pos_dict = dict(zip(msg.name, msg.position))
        vel_dict = dict(zip(msg.name, msg.velocity)) if msg.velocity else {}

        for i, name in enumerate(self.joint_names):
            q[i] = pos_dict.get(name, 0.0)
            q_dot[i] = vel_dict.get(name, 0.0)

        try:
            if self.use_linearized:
                tau = self.compensator.compute_gravity_compensation_linearized(
                    q=q,
                    q_dot=q_dot,
                    grav_gain=self.grav_gain,
                    fric_gain=self.fric_gain
                )
            else:
                tau = self.compensator.compute_gravity_compensation_pinocchio(q)

            # 直接发布
            msg_out = Float64MultiArray()
            msg_out.data = tau.tolist()
            self.pub_effort.publish(msg_out)

            # 可选：偶尔打印一下看看
            #self.get_logger().info(f"tau: {tau}")

        except Exception as e:
            self.get_logger().error(f"计算失败: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = GravityCompensationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"发生错误: {e}")
    finally:
        # 直接 shutdown，不调用 destroy_node 避免奇怪错误
        rclpy.shutdown()


if __name__ == '__main__':
    main()