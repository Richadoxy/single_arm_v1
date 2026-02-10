#include "robot_arm.h"
#include "can_normal.h"
#include "can_fd.h"

RobotArm::RobotArm(const Robot& robot)
    : robot_config_(robot), can_channel_(robot.motors.can_channel) {
    motors_.resize(num_joints_);

    for (int i = 0; i < num_joints_; ++i) {
        uint32_t id = robot.motors.ids[i];
        MotorType type = robot.motors.types[i];  // 从配置读取类型

        motor_enable_list_[i] = robot.motors.enabled[i];
        joint_offsets_[i] = robot.mdh.init_mdhpos[i];  // 假设 Robot 有这个字段
        joint_signs_[i] = robot.mdh.sign[i];     // 1 或 -1
        if (type == MotorType::WHJ) {
            auto motor = std::make_unique<WhjMotor>(id, can_channel_);
            whj_motor_map_[id] = motor.get();
            motors_[i] = std::move(motor);
        } else if (type == MotorType::RMD) {
            auto motor = std::make_unique<RmdMotor>(id, can_channel_);
            rmd_motor_map_[id] = motor.get();
            motors_[i] = std::move(motor);
        } else {
            std::cerr << "Unknown motor type for joint " << i << std::endl;
        }
    }
    for (int i = 0; i < num_sensors_;++i){
        tqe_sensor_enable_list_[i] = robot.sensors.enabled[i];
        uint32_t id = robot.sensors.ids[i]; 
        new (&tqe_sensor_[i]) TorqueSensor(id, can_channel_);
        tqe_sensor_map_[id] = &tqe_sensor_[i];   
        torque_signs[i] = robot.sensors.sign[i];
    }
}

RobotArm::~RobotArm() {
    // Cleanup resources
}

int RobotArm::poll_canfd_responses(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    int received_flag = 0;

    while (!received_flag) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) break;

        uint32_t respId = 0;
        uint8_t respMaxData[64] = {0};
        uint8_t respDlc = 0;
        uint8_t receiveStatus = canfd_receive(can_channel_, 0, &respId, respMaxData, &respDlc, 1);
        if (receiveStatus < 0) continue;

        // 通过 motor_id 查找（Whj ID 通常低字节区分）
        auto it = whj_motor_map_.find(respId & 0xFF);
        if (it != whj_motor_map_.end()) {
            MotorInterface* motor = it->second;
            // 统一调用虚函数 decode_feedback（推荐方式）
            motor->decode_feedback(respId, respMaxData, respDlc);
            received_flag = 1;
        }
    }

    if (!received_flag) {
        std::cerr << "WhjMotor: No CAN-FD responses in " << timeout_ms << " ms" << std::endl;
        return -1;
    }
    return 0;
}

int RobotArm::poll_can_responses(int timeout_ms) {
    auto start_time = std::chrono::steady_clock::now();
    int received_flag = 0;

    while (!received_flag) {
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::steady_clock::now() - start_time).count();
        if (elapsed >= timeout_ms) break;

        uint32_t respId = 0;
        uint8_t respMaxData[8] = {0};
        uint8_t respDlc = 0;
        uint8_t receiveStatus = can_normal_receive(can_channel_, respMaxData, &respId);
        //std::cout << "RobotArm poll_can_responses received ID: " << std::hex << respId << std::dec << std::endl;

        auto it = rmd_motor_map_.find(respId - 0x240);
        if (it != rmd_motor_map_.end()) {
            MotorInterface* motor = it->second;
            // 统一调用虚函数 decode_feedback
            motor->decode_feedback(respId, respMaxData, respDlc);
            received_flag = 1;
        }
        auto sit = tqe_sensor_map_.find(respId - 0x100);
        if (sit != tqe_sensor_map_.end() && sit->second != nullptr) {
            sit->second->decode_sensor_data(respMaxData);
            received_flag = 1;
        }
    }

    if (!received_flag) {
        std::cerr << " No CAN responses in " << timeout_ms << " ms" << std::endl;
        return -1;
    }
    return 0;
}
double RobotArm::mdh_to_joints(int index, const double mdh_rad) {
    double joints_rad = joint_offsets_[index] + joint_signs_[index] * mdh_rad;
    return joints_rad;
}

double RobotArm::joints_to_mdh(int index, const double joints_rad) {
    double mdh_rad = (joint_signs_[index] * (joints_rad - joint_offsets_[index]));
    return mdh_rad;
}

int RobotArm::initialize(int max_retry_times) { 
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        int retry_count = 0;
        bool success = false;    
        MotorInterface* motor = motors_[joint].get();
        if (motor->get_type() != MotorType::WHJ) {
            while (!success && retry_count < max_retry_times) {
                motor->set_velocity(0.0);
                poll_can_responses(10);
                if (motor->is_control_success()) {
                    std::cout << "Joint " << joint << " Enable success" << std::endl;
                    success = true;
                } else {
                    std::cout << "Joint " << joint << " Enable failed, retrying .... " << retry_count << std::endl;
                }
                retry_count++;
            }
            retry_count = 0;
            success = false;
        }else{
            while (!success && retry_count < max_retry_times)
                    {
                        if (motor->set_IAP() == 0) {
                            poll_canfd_responses(10);
                            if (motor->is_control_success()) {
                                success = true;
                                std::cout << "Joint " << joint << "  set_IAP success" << std::endl;
                            } else {
                                std::cout << "Joint " << joint << "  set_IAP failed, retrying .... " << retry_count << std::endl;
                            }
                        } else {
                            std::ostringstream oss;
                            std::cout << "JOINT " << joint << "  sending IAP cmd failed, check wire connection" << retry_count << std::endl;
                            return -1;
                        }
                        retry_count++;
                    }
                    success = false;
                    retry_count = 0;
                    while (!success && retry_count < max_retry_times) {
                        if (motor->enable() == 0) {
                            poll_canfd_responses(10);
                            if (motor->is_control_success()) {
                                success = true;
                                std::cout << "Joint " << joint << " (WHJ) enable success" << std::endl;
                            } else {
                                std::cout << "Joint " << joint << " (WHJ) enable failed, retrying .... " << retry_count << std::endl;
                            }
                        } else {
                            std::ostringstream oss;
                            std::cout << "JOINT " << joint << " (WHJ) sending enable cmd failed, check wire connection" << retry_count << std::endl;
                            return -1;
                        }
                    }
        }
        
        
        
        
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
        
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return 0;
}

int RobotArm::shut_down() {
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        motors_[joint]->disable();
        if (motors_[joint]->get_type() == MotorType::WHJ) {
            poll_canfd_responses(10);
        } else {
            poll_can_responses(10);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}

int RobotArm::set_control_mode(CONTROL_MODE mode) {
    int max_retry_times = 100;
    int retry_count = 0;
    bool success = false;
    double zero_mdh_pos[num_joints_] = {0.0};
    for (int joint = 0; joint < num_joints_; ++joint) {
        int retry_count = 0;
        bool success = false;
        if (motor_enable_list_[joint] == 0) 
            continue;
        while (!success && retry_count < max_retry_times) {
            if (motors_[joint]->select_mode(mode) == 0) {
                if (motors_[joint]->get_type() != MotorType::WHJ) {
                    poll_can_responses(10);
                }
                else
                {
                    poll_canfd_responses(10);
                }
                if (motors_[joint]->is_control_success()) {
                    success = true;
                    std::cout << "Joint " << joint << "  select_mode success" << std::endl;
                } else {
                    std::cout << "Joint " << joint << "  select_mode failed, retrying .... " << retry_count << std::endl;
                }
            } else {
                std::ostringstream oss;
                std::cout << "JOINT " << joint << "sending select_mode cmd failed, check wire connection" << retry_count << std::endl;
                return -1;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return 0;
}


int RobotArm::set_mdh_position(int index, double mdh_pos) {
    double joint_pos = mdh_to_joints(index, mdh_pos);
    if (motors_[index]->set_absposition(joint_pos) !=0)
        return -1;
    if (motors_[index]->get_type() == MotorType::WHJ) 
        poll_canfd_responses();
    else
        poll_can_responses();
    return 0;
}

int RobotArm::set_velocity(int index, double vel_rad_s) {
    if (motors_[index]->set_velocity(vel_rad_s * joint_signs_[index]) != 0)
        return -1;
    if (motors_[index]->get_type() == MotorType::WHJ) 
        poll_canfd_responses();
    else
        poll_can_responses();
    return 0;
}

int RobotArm::set_current(int index, double cur_A) {
    if (motors_[index]->set_current(cur_A * joint_signs_[index]) != 0)
        return -1;
    if (motors_[index]->get_type() == MotorType::WHJ) 
        poll_canfd_responses();
    else
        poll_can_responses();
    return 0;
}
int RobotArm::read_torque_sensor(int index) {
    if (index < 0 || index >= num_sensors_ ) {
        std::cout << "[Sensor] index out of range" << std::endl;
        return -1;
    }
    int ret = tqe_sensor_[index].read_sensor();
    poll_can_responses();
    return ret;
}
int RobotArm::read_status(int index) {   
    if (motors_[index]->read_status() != 0)
        return -1;
    if (motors_[index]->get_type() == MotorType::WHJ) 
        poll_canfd_responses();
    else
        poll_can_responses();
    // 更新本地缓存
    return 0;
}

double RobotArm::get_mdh_position(int index) {
    double mdh_pos = joints_to_mdh(index, motors_[index]->get_position());
    return mdh_pos;
}

double RobotArm::get_velocity(int index) {
    return motors_[index]->get_velocity() * joint_signs_[index];
}

double RobotArm::get_current(int index) {
    return motors_[index]->get_current() * joint_signs_[index];
}

double RobotArm::get_torque(int index) {
    if (index < 0 || index >= num_sensors_ ) {
        std::cout << "[Sensor] index out of range" << std::endl;
        return 0.0;
    }
    return tqe_sensor_[index].get_torque() * torque_signs[index];
}
int RobotArm::move_joint(double *target_joints, double dt, double T) 
{
    FILE* fwrite_data = fopen("trajectory_data.txt", "w");
    if (fwrite_data == nullptr) {
        std::cerr << "Failed to open file: trajectory_data.txt" << std::endl;
        return -1;
    }
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (target_joints[joint] < robot_config_.mdh.limit_min[joint] ||
            target_joints[joint] > robot_config_.mdh.limit_max[joint]) {
            std::cerr << "Target joint " << joint << " position " << target_joints[joint]
                      << " out of limits [" << robot_config_.mdh.limit_min[joint]
                      << ", " << robot_config_.mdh.limit_max[joint] << "]" << std::endl;
            return -1;
        }
    }
    double start_pos[num_joints_];
    std::vector<double> traj_coef[num_joints_];
    int stop_flag = 0;
    for (int joint = 0; joint < num_joints_; ++joint) {
        if (motor_enable_list_[joint] == 0) 
            continue;
        read_status(joint);
        start_pos[joint] = get_mdh_position(joint);
        std::cout << "Joint " << joint << " start pos: " << start_pos[joint]
                  << ", target pos: " << target_joints[joint] << std::endl;
        traj_coef[joint] = polynomial_trajectory::calculateCoefficients(start_pos[joint], 0.0, 0.0, 
                                                                        target_joints[joint], 0.0, 0.0, T);                                                 
    }
    
    
    double t = dt;
    while (t <= T)
    {
        auto cycle_start = std::chrono::steady_clock::now();
        for (int joint = 0; joint < num_joints_; ++joint) {
            if (motor_enable_list_[joint] == 0) 
                continue;
            double ref_pos, vel, acc;
            std::tie(ref_pos, vel, acc) = polynomial_trajectory::evaluatePolynomial(traj_coef[joint], t);
            if (set_mdh_position(joint, ref_pos) != 0) {
                std::cerr << "Failed to set position for joint " << joint << std::endl;
                return -1;
            }
            if (motors_[joint]->get_type() != MotorType::WHJ) 
                read_status(joint);
            
            // if ((int)(t / dt) % 10 == 0) {
            //     std::cout <<"joint " << joint << " | target pos: " << ref_pos
            //                 << " | act pos: " << get_mdh_position(joint) << " | act vel: " << get_velocity(joint) 
            //                 << " | act cur: " << get_current(joint)  << std::endl;
            // }    
            fprintf(fwrite_data, "%.4f %.4f %.4f %.4f ", 
                    ref_pos, get_mdh_position(joint), get_velocity(joint), get_current(joint));
        }

        auto cycle_end = std::chrono::steady_clock::now();
        auto elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end - cycle_start).count();
        long target_cycle_us = static_cast<long>(dt * 1000000);  // 10ms = 10000us
        long remaining_us = target_cycle_us - elapsed_us;
    
        // 打印详细的耗时信息
        std::cout << "Cycle time: " << elapsed_us << "us, "
              << "Target: " << target_cycle_us << "us, "
              << "Remaining: " << remaining_us << "us" << std::endl;

        
        if (remaining_us > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(remaining_us));
        } else {
            std::cerr << "Warning: Cycle " << (t/dt) << " took too long! " 
                    << elapsed_us << "us > " << target_cycle_us << "us" << std::endl;
        }
        auto actual_cycle_end = std::chrono::steady_clock::now();
        double actual_dt = std::chrono::duration<double>(actual_cycle_end - cycle_start).count();
        t += actual_dt;
        fprintf(fwrite_data, "%ld %ld %ld %.6f \n", 
            elapsed_us, remaining_us, target_cycle_us, actual_dt);
    }
    
    return 0;
}