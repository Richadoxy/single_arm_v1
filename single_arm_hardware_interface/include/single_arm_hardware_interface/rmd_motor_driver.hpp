// rmd_motor_driver.hpp
#pragma once
#ifndef RMD_MOTOR_DRIVER_HPP__
#define RMD_MOTOR_DRIVER_HPP__
#include "single_arm_hardware_interface/motor_driver_base.hpp"

namespace single_arm_hardware_interface
{

class RmdMotorDriver : public MotorDriver
{
public:
  RmdMotorDriver() = default;
  virtual ~RmdMotorDriver() = default;

  bool parse_parameters(const hardware_interface::ComponentInfo& joint) override;

  bool send_iap() override;
  bool send_clear_error() override;
  bool send_set_mode(IntegrationLevel level) override;
  bool send_enable(bool enable) override;
  bool read_status() override;
  void write_position_command(double position_cmd_rad) override;
  void write_effort_command(double effort_cmd) override;
  void write_velocity_command(double effort_cmd);  // 统一参数为 effort_cmd

  // 只实现 classic (RMD 用 can0 classic)
  bool process_classic_frame(const Frame::SharedPtr& msg,
                                     double& out_position_rad,
                                     double& out_velocity_rad_s,
                                     double& out_effort,
                                     uint16_t& out_enable_state,
                                     uint16_t& out_error_code)
  override;

private:
  // RMD 协议专用常量（基于您提供的 motor.cpp）
  static constexpr uint32_t CMD_DISABLE       = 0x80;
  static constexpr uint32_t CMD_READ_STATUS   = 0x92;
  static constexpr uint32_t CMD_TORQUE        = 0xA1;
  static constexpr uint32_t CMD_VELOCITY      = 0xA2;
  static constexpr uint32_t CMD_POSITION      = 0xA4;

  static constexpr uint32_t FEEDBACK_ID_OFFSET = 0x100;  // 反馈 ID = motor_id + 0x100 → 实际为 +0x240 - 0x140

  // 底层发送 8 字节命令（RMD 经典 CAN，len=8）
  void send_rmd_command(uint8_t cmd, const std::vector<uint8_t>& payload = {});

  // 目标值转换（与原 motor.cpp 一致）
  int32_t get_target_position_units(double pos_rad) const;
  int16_t get_target_current_units(double effort_a) const;  // effort 为 A
};

}  // namespace single_arm_hardware_interface

#endif  // RMD_MOTOR_DRIVER_HPP__