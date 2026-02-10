#include <iostream>
#include <iomanip>
#include <thread>
#include <signal.h>
#include <fstream>
#include <chrono>
#include <vector>
#include <cmath>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "config_loader.h"
#include "can_fd.h"
#include "can_normal.h"
#include "robot_arm.h"
// Function to check if a key is pressed (non-blocking)

int main(int argc, char *argv[]) {
    // Load configuration from config.yaml
    std::string base_path = PROJECT_SOURCE_DIR;
    ConfigLoader config_loader(base_path + "/config.yaml");
    const auto& robot_config = config_loader.getRobot();
    // 初始化CAN
    can_normal_init(0);
    canfd_init(0, 0);
    RobotArm single_arm_v1(robot_config);
    single_arm_v1.initialize(100);
    single_arm_v1.set_control_mode(cur_control);
    for (int i = 0; i < 1000; ++i) {
        for (int j = 0; j < 6; ++j) {
            if (robot_config.motors.enabled[j] == 0)
                continue;
            single_arm_v1.read_status(j);
            std::cout << "Joint " << (j + 1) << " Position: " << std::fixed << std::setprecision(4)
                      << single_arm_v1.get_mdh_position(j) << " rad, Velocity: "
                      << single_arm_v1.get_velocity(j) << " rpm, Current: "
                      << single_arm_v1.get_current(j) << " mA" << std::endl;
            
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    can_normal_close(0);
    canfd_close(0, 0);
    return 0;
}