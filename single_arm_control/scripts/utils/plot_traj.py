#!/usr/bin/env python3
"""
读取 recorded_trajectory_*.txt 文件并绘制对比图
用法：
    python3 plot_recorded_trajectory.py recorded_trajectory_20260127_214641.txt
    或直接修改下面的 filename 变量
"""

import sys
import numpy as np
import matplotlib.pyplot as plt

def load_data(filename):
    """读取 txt 文件，返回各个数组"""
    try:
        data = np.loadtxt(filename, delimiter=",")
    except Exception as e:
        print(f"读取文件失败: {e}")
        sys.exit(1)
    
    if data.shape[1] != 37:
        print(f"列数不匹配！预期 37 列，实际 {data.shape[1]} 列")
        sys.exit(1)
    
    t       = data[:, 0]              # 时间戳 (col 0)
    q       = data[:, 1:7]            # actual position   1~6
    dq      = data[:, 7:13]           # actual velocity   7~12
    tau     = data[:, 13:19]          # actual effort     13~18
    qr      = data[:, 19:25]          # reference position 19~24
    dqr     = data[:, 25:31]          # reference velocity 25~30
    ddqr    = data[:, 31:37]          # reference acc      31~36
    
    return t, q, dq, tau, qr, dqr, ddqr


def plot_trajectory(filename):
    t, q, dq, tau, qr, dqr, ddqr = load_data(filename)
    
    joint_names = [f'joint{i+1}' for i in range(6)]
    
    # 图1：位置对比 qr vs q
    fig1, axes1 = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    fig1.suptitle('Position Tracking: Reference (qr) vs Actual (q)', fontsize=16)
    
    for i, ax in enumerate(axes1.flat):
        ax.plot(t, qr[:, i], 'b-', label='qr (reference)', linewidth=1.5)
        ax.plot(t, q[:, i],  'r--', label='q (actual)', linewidth=1.2)
        ax.set_title(joint_names[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.grid(True)
        ax.legend(loc='upper right', fontsize='small')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    
    # 图2：速度对比 dqr vs dq
    fig2, axes2 = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    fig2.suptitle('Velocity Tracking: Reference (dqr) vs Actual (dq)', fontsize=16)
    
    for i, ax in enumerate(axes2.flat):
        ax.plot(t, dqr[:, i], 'b-', label='dqr (reference)', linewidth=1.5)
        ax.plot(t, dq[:, i],  'r--', label='dq (actual)', linewidth=1.2)
        ax.set_title(joint_names[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.grid(True)
        ax.legend(loc='upper right', fontsize='small')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    
    # 图3：电流（effort / tau）对比
    # 注意：这里没有参考电流，所以只画实际 tau
    fig3, axes3 = plt.subplots(2, 3, figsize=(15, 8), sharex=True)
    fig3.suptitle('Actual Joint Effort (Current / Torque)', fontsize=16)
    
    for i, ax in enumerate(axes3.flat):
        ax.plot(t, tau[:, i], 'g-', label='tau (actual effort)', linewidth=1.5)
        ax.set_title(joint_names[i])
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Effort (Nm or A)')
        ax.grid(True)
        ax.legend(loc='upper right', fontsize='small')
    
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        # 如果不传参数，可以在这里手动指定文件名
        filename = "recorded_trajectory_20260206_130350.txt"  # ← 修改成你的实际文件名
    
    print(f"正在读取文件: {filename}")
    plot_trajectory(filename)