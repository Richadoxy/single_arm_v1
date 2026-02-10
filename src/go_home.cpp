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
    canfd_init(0,0);
    RobotArm single_arm_v1(robot_config);
    single_arm_v1.initialize(100);
    single_arm_v1.set_control_mode(pos_control);
    double home[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    double target_joints_1[6] = {-0.5186, 1.37442,-0.97, 1.4, 1.046, -1.3};
    single_arm_v1.move_joint(home,0.01, 3);
    return 0;
}