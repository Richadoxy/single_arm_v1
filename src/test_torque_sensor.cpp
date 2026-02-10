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
    FILE* fwrite_data = fopen("torque_sensor_data.txt", "w");
    double zero_read[6] = {0.0};
    for (int j = 0;j<3;++j)
    {
        if (robot_config.sensors.enabled[j] == 0)
                continue;
        single_arm_v1.read_torque_sensor(j);
        zero_read[j] = single_arm_v1.get_torque(j);

    }
    for (int i = 0; i < 1000; ++i) {
        for (int j = 0; j < 3; ++j) {
            if (robot_config.sensors.enabled[j] == 0)
                continue;
            single_arm_v1.read_torque_sensor(j);
            single_arm_v1.read_status(j);
            std::cout << "Joint " << (j + 1) << " Toruqe: " << std::fixed << std::setprecision(4)
                      << single_arm_v1.get_torque(j) -zero_read[j]<< " N/m: " << "Current: " << single_arm_v1.get_current(j) << " A " << std::endl;
            fprintf(fwrite_data, "%.4f ", single_arm_v1.get_torque(j));
        }
        fprintf(fwrite_data, "\n");
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    can_normal_close(0);
    fclose(fwrite_data);
    return 0;
}