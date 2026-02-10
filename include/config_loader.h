#pragma once
#ifndef CONFIG_LOADER_H
#define CONFIG_LOADER_H

#include <yaml-cpp/yaml.h>
#include <vector>
#include <stdexcept>
#include <string>
#include "robot_struct.h"  // 包含使用 std::vector 的 Robot 定义

class ConfigLoader {
public:
    explicit ConfigLoader(const std::string& filename);
    const Robot& getRobot() const { return robot_; }

private:
    Robot robot_;

    void loadFromFile(const std::string& filename);

    // 辅助函数：复制 YAML 数组到 vector，并 resize + 严格检查大小
    template<typename T>
    void copyVector(const YAML::Node& yaml_node, std::vector<T>& dest, size_t expected_size) {
        if (!yaml_node) {
            throw std::runtime_error("Missing YAML node");
        }
        if (yaml_node.size() != expected_size) {
            throw std::runtime_error("YAML array size mismatch: expected " +
                                     std::to_string(expected_size) +
                                     ", got " + std::to_string(yaml_node.size()));
        }
        dest.resize(expected_size);
        for (size_t i = 0; i < expected_size; ++i) {
            dest[i] = yaml_node[i].as<T>();
        }
    }

    // 固定大小的专用版本（如 base_to_world、tool、cartesian_command 固定为 6）
    template<typename T>
    void copyVectorFixed(const YAML::Node& yaml_node, std::vector<T>& dest, size_t fixed_size = 6) {
        if (!yaml_node || yaml_node.size() != fixed_size) {
            throw std::runtime_error("Fixed size array mismatch (expected " +
                                     std::to_string(fixed_size) + ")");
        }
        dest.resize(fixed_size);
        for (size_t i = 0; i < fixed_size; ++i) {
            dest[i] = yaml_node[i].as<T>();
        }
    }
};

// ==================== 实现部分 ====================

ConfigLoader::ConfigLoader(const std::string& filename) {
    loadFromFile(filename);
}

void ConfigLoader::loadFromFile(const std::string& filename) {
    YAML::Node config = YAML::LoadFile(filename);

    // 1. 首先读取关节数（必须最先读取，用于后续所有 resize 和验证）
    size_t num_joints = config["robot"]["num_joints_per_arm"].as<size_t>();
    size_t num_sensors = config["robot"]["num_sensors"].as<size_t>();
    robot_.num_joints_ = num_joints;

    // 2. 解析 kinematics（所有关节相关数组必须长度 == num_joints）
    const YAML::Node& kin_node = config["kinematics"];
    copyVector(kin_node["a"], robot_.mdh.a, num_joints);
    copyVector(kin_node["alpha"], robot_.mdh.alpha, num_joints);
    copyVector(kin_node["d"], robot_.mdh.d, num_joints);
    copyVector(kin_node["sign"], robot_.mdh.sign, num_joints);
    copyVector(kin_node["offset"], robot_.mdh.offset, num_joints);
    copyVector(kin_node["init_mdhpos"], robot_.mdh.init_mdhpos, num_joints);
    copyVector(kin_node["limits"]["min"], robot_.mdh.limit_min, num_joints);
    copyVector(kin_node["limits"]["max"], robot_.mdh.limit_max, num_joints);

    // 固定大小数组
    copyVectorFixed(kin_node["base_to_world"], robot_.mdh.base_to_world);
    copyVectorFixed(kin_node["tool"], robot_.mdh.tool);

    // 3. 解析 dynamics（固定 28 个参数 = num_joints * 4）
    const YAML::Node& dyn_node = config["dynamics"];
    size_t expected_dyn_size = num_joints * 4;
    copyVector(dyn_node["dynamics_parm"], robot_.dynamics.dynamics_parm, expected_dyn_size);

    // 4. 解析 hardware
    const YAML::Node& hw_node = config["hardware"];

    // motor 参数
    const YAML::Node& motor_node = hw_node["motor"];
    const YAML::Node& sensor_node = hw_node["torque_sensor"];
    copyVector(motor_node["ids"], robot_.motors.ids, num_joints);
    copyVector(motor_node["enabled"], robot_.motors.enabled, num_joints);
    copyVector(motor_node["gear_ratios"], robot_.motors.gear_ratios, num_joints);
    copyVector(motor_node["rated_torque"], robot_.motors.rated_torque, num_joints);
    copyVector(motor_node["max_velocity_rpm"], robot_.motors.max_velocity_rpm, num_joints);
    copyVector(motor_node["max_current_mA"], robot_.motors.max_current_mA, num_joints);
    copyVector(motor_node["max_position_step"], robot_.motors.max_position_step, num_joints);

    copyVector(sensor_node["ids"], robot_.sensors.ids, num_sensors);
    copyVector(sensor_node["enabled"], robot_.sensors.enabled, num_sensors);
    copyVector(sensor_node["sign"], robot_.sensors.sign, num_sensors);
    // motor_type：字符串数组 → MotorType 枚举
    const YAML::Node& type_node = motor_node["motor_type"];
    if (!type_node || type_node.size() != num_joints) {
        throw std::runtime_error("motor_type array size mismatch");
    }
    robot_.motors.types.resize(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
        std::string type_str = type_node[i].as<std::string>();
        if (type_str == "WHJ") {
            robot_.motors.types[i] = MotorType::WHJ;
        } else if (type_str == "RMD") {
            robot_.motors.types[i] = MotorType::RMD;
        } else {
            throw std::runtime_error("Unknown motor_type: " + type_str);
        }
    }

    // CAN 配置（可选，默认值已在 robot_struct.h 中设置）
    const YAML::Node& can_node = hw_node["can"];
    robot_.motors.can_channel = can_node["channel"].as<int>(robot_.motors.can_channel);
    robot_.motors.can_baudrate = can_node["baudrate"].as<int>(robot_.motors.can_baudrate);

    const YAML::Node& canfd_node = hw_node["canfd"];
    robot_.motors.canfd_channel = canfd_node["channel"].as<int>(robot_.motors.canfd_channel);
    robot_.motors.canfd_baudrate = canfd_node["baudrate"].as<int>(robot_.motors.canfd_baudrate);

    // 5. 解析 control
    const YAML::Node& ctrl_node = config["control"];

    // position
    const YAML::Node& pos_node = ctrl_node["position"];
    robot_.control.position.dt = pos_node["dt"].as<double>();
    robot_.control.position.T = pos_node["T"].as<double>();
    copyVector(pos_node["command_degrees"], robot_.control.position.command_degrees, num_joints);
    copyVectorFixed(pos_node["cartesian_command"], robot_.control.position.cartesian_command);

    // velocity
    const YAML::Node& vel_node = ctrl_node["velocity"];
    robot_.control.velocity.interpolation_dt = vel_node["interpolation_dt"].as<double>();
    robot_.control.velocity.interpolation_T = vel_node["interpolation_T"].as<int>();
    robot_.control.velocity.error_tolerance = vel_node["error_tolerance"].as<double>();
    copyVector(vel_node["kp"], robot_.control.velocity.kp, num_joints);
    copyVector(vel_node["max_velocity"], robot_.control.velocity.max_velocity, num_joints);
    copyVector(vel_node["command_degrees"], robot_.control.velocity.command_degrees, num_joints);
}

#endif // CONFIG_LOADER_H