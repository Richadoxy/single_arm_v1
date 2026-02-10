#ifndef ROBOT_ARM_H
#define ROBOT_ARM_H

#include <map>
#include <unordered_map>
#include <vector>
#include <chrono>
#include <thread>
#include <iostream>
#include <fstream>
#include "motor.h"
#include "polynomial_trajectory.h"
#include "robot_struct.h"
#include "torque_sensor.h"

class RobotArm {
public:
    static const int num_joints_ = 6;
    static const int num_sensors_ = 3;
    RobotArm(const Robot& robot= Robot());
    ~RobotArm();
    // 初始化与关闭
    int initialize(int max_retry_times);
    int shut_down();
    int set_control_mode(CONTROL_MODE mode);

    // 单关节插值控制接口
    int set_mdh_position(int index, double mdh_pos);
    int set_velocity(int index, double vel_rad_s);
    int set_current(int index, double cur_A);
    int read_status(int index);
    int read_torque_sensor(int index);
    // 读取接口
    double get_mdh_position(int index);
    double get_velocity(int index);
    double get_current(int index);
    double get_torque(int index);
    void get_all_positions(double positions[6]) const;
    void get_all_velocities(double velocities[6]) const;
    void get_all_currents(double currents[6]) const;
    int get_end_effector_pose(double pose[6]);
    
    // 运动控制接口
    int move_joint(double *target_joints, double dt, double T);
    int move_line(const double* target_cart, double dt, double T);

private:
    std::vector<std::unique_ptr<MotorInterface>> motors_;
    std::unordered_map<uint32_t, MotorInterface*> whj_motor_map_; 
    std::unordered_map<uint32_t, MotorInterface*> rmd_motor_map_;  
    std::unordered_map<uint32_t, TorqueSensor*> tqe_sensor_map_;
    double joint_offsets_[num_joints_];
    int joint_signs_[num_joints_];
    int torque_signs[num_sensors_];
    int can_channel_;
    int motor_enable_list_[num_joints_];
    int tqe_sensor_enable_list_[num_sensors_];
    WhjMotor whj_motors_[3];
    RmdMotor rmd_motors_[3];
    TorqueSensor tqe_sensor_[3];
    int poll_canfd_responses(int timeout_ms = 10);
    int poll_can_responses(int timeout_ms = 10);
    double mdh_to_joints(int index, const double mdh_degrees);
    double joints_to_mdh(int index, const double joints_degrees);
    const Robot& robot_config_;  
};

#endif