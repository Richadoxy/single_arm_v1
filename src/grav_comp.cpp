#include <iostream>
#include <iomanip>
#include <thread>
#include <chrono>
#include <vector>
#include <atomic>
#include <csignal>
#include "config_loader.h"
#include "can_fd.h"
#include "can_normal.h"
#include "robot_arm.h"
#include "ccg_controller.h"

// 全局标志，用于 Ctrl+C 退出
std::atomic<bool> running(true);

void signal_handler(int sig) {
    if (sig == SIGINT || sig == SIGTERM) {
        std::cout << "\n收到退出信号，准备关闭...\n";
        running = false;
    }
}

int main(int argc, char *argv[]) {
    // 注册 Ctrl+C 信号处理
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    FILE* fwrite_data = fopen("grav_comp.txt", "w");
    // 加载配置
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robot_config = config_loader.getRobot();

    // 初始化 CAN
    can_normal_init(0);
    canfd_init(0, 0);

    // 初始化机械臂
    RobotArm single_arm_v1(robot_config);
    single_arm_v1.initialize(100);
    single_arm_v1.set_control_mode(cur_control);  // ← 这里需要确认 cur_control 已定义

    // 创建 CCG 控制器
    CCGController ccg_controller;

    std::cout << "进入主控制循环，按 Ctrl+C 退出...\n";
    std::cout << std::fixed << std::setprecision(4);

    // 主循环
    while (running) {
        // 1. 获取当前关节状态
        std::vector<double> q(6), q_dot(6);
        std::vector<double> act_cur(6);
        for (int j = 0; j < 6; ++j) {
            q[j]     = single_arm_v1.get_mdh_position(j);     // 位置 (rad)
            q_dot[j] = single_arm_v1.get_velocity(j); // 
            act_cur[j] = single_arm_v1.get_current(j);
            printf("joint %d          q: %f          qd: %f          cur: %f\n", j+1, q[j], q_dot[j], act_cur[j]);
            // 注意：如果你的 get_velocity 返回的是 rpm，需要转换为 rad/s
        }

        // 2. 计算 CCG 补偿扭矩（单位：Nm）
        std::vector<double> tau_comp(6);
        ccg_controller.computeTau(q, q_dot, tau_comp);

        // 3. 根据硬件把扭矩转换为电流指令（示例：简单线性映射）
        //    请根据你的电机驱动器实际关系替换这个转换

        std::vector<double> current_cmd(6);
        for (int j = 0; j < 6; ++j) {
            current_cmd[j] = tau_comp[j];
            if (j<3)
                current_cmd[j] *= 0.3;
            // 可加限幅
        }

        // 4. 发送电流/扭矩指令到每个关节
        for (int j = 0; j < 6; ++j) {
            single_arm_v1.set_current(j, current_cmd[j]);
            fprintf(fwrite_data, "%f %f %f %f ", q[j], q_dot[j], act_cur[j], current_cmd[j]);
        }
        fprintf(fwrite_data, "\n");
        // 5. 打印状态（可选每 10 次打印一次，避免刷屏太快）

        // 控制周期 ≈ 10ms
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    std::cout << "退出主循环，正在关闭...\n";
    single_arm_v1.set_control_mode(pos_control);
    // 清理
    // 可选：发送零电流或进入安全模式
    
    std::this_thread::sleep_for(std::chrono::milliseconds(200));

    can_normal_close(0);
    canfd_close(0, 0);
    fclose(fwrite_data);
    std::cout << "程序正常退出\n";
    return 0;
}