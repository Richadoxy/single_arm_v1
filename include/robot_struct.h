#pragma once
#ifndef ROBOT_STRUCT_H_
#define ROBOT_STRUCT_H

#include <vector>
#include <cstdint>
#include "motor_interface.h"

constexpr double PI = 3.141592653589793;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;

struct MDH {
    std::vector<double> a;
    std::vector<double> alpha;
    std::vector<double> d;
    std::vector<double> sign;
    std::vector<double> offset;
    std::vector<double> init_mdhpos;
    std::vector<double> limit_min;
    std::vector<double> limit_max;

    std::vector<double> base_to_world;  // 固定6
    std::vector<double> tool;           // 固定6
};

struct Dynamics {
    std::vector<double> dynamics_parm;  // 运行时大小 = num_joints * 4
};

struct MotorParams {
    std::vector<uint32_t> ids;
    std::vector<int> enabled;
    std::vector<double> gear_ratios;
    std::vector<double> rated_torque;
    std::vector<double> max_velocity_rpm;
    std::vector<double> max_current_mA;
    std::vector<double> max_position_step;
    std::vector<MotorType> types;

    // CAN 配置保持不变
    int can_channel = 0;
    int can_baudrate = 1000000;
    int canfd_channel = 0;
    int canfd_baudrate = 5000000;
};
struct SensorParams {
    std::vector<uint32_t> ids;
    std::vector<int> enabled;
    std::vector<double> sign;

    // CAN 配置保持不变
    int can_channel = 0;
};
struct PositionControl {
    double dt = 0.01;
    double T = 5.0;
    std::vector<double> command_degrees;
    std::vector<double> cartesian_command;  // 固定6
};

struct VelocityControl {
    double interpolation_dt = 0.01;
    int interpolation_T = 10;
    std::vector<double> kp;
    std::vector<double> max_velocity;
    double error_tolerance = 50.0;
    std::vector<double> command_degrees;
};

struct ControlParams {
    PositionControl position;
    VelocityControl velocity;
};

class Robot {
public:
    Robot() = default;

    size_t num_joints() const { return num_joints_; }  // 方便外部获取

    MDH mdh;
    Dynamics dynamics;
    MotorParams motors;
    ControlParams control;
    SensorParams sensors;
private:
    size_t num_joints_ = 0;  // 由 ConfigLoader 在加载后设置
    friend class ConfigLoader;  // 让 loader 可以访问私有成员
};

#endif