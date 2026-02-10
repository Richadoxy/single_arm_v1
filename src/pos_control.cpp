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
    single_arm_v1.set_control_mode(pos_control);
    const int num_points =5;
    double target_joints[num_points][6] = {
        {0.315, 0.66, 0.66, 0.46, 0.80, -0.0},
        {-0.0569, 1.39, -0.47, 0.28, -1.11, 0.0},
        {-0.5186, 1.37442,-0.97, 0.22, 1.046, 0.0},
        {-0.2186, 0.37442,-0.5, 0.1, 0.546, 0.0},
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    };


    for (int i = 0; i <5; ++i) {
        single_arm_v1.move_joint(target_joints[i],0.01, 2);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
    

    return 0;
}