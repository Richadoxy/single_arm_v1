// whj_motor_driver.hpp
#pragma once
#ifndef WHJ_MOTOR_DRIVER_HPP__
#define WHJ_MOTOR_DRIVER_HPP__

#include "single_arm_hardware_interface/motor_driver_base.hpp"
#include "single_arm_hardware_interface/motor_command.hpp"

namespace single_arm_hardware_interface
{

class WHJMotorDriver : public MotorDriver
{
public:
  WHJMotorDriver() = default;
  virtual ~WHJMotorDriver() = default;

  bool parse_parameters(const hardware_interface::ComponentInfo& joint) override;
  bool send_iap() override;
  bool send_clear_error() override;
  bool send_set_mode(IntegrationLevel level) override;
  bool send_enable(bool enable) override;
  bool read_status() override;
  void write_position_command(double position_cmd_rad) override;
  void write_effort_command(double effort_cmd) override;

  bool process_fd_frame(const FdFrame::SharedPtr& msg,
                     double& out_position_rad,
                     double& out_velocity_rad_s,
                     double& out_effort,
                     uint16_t& out_enable_state,
                     uint16_t& out_error_code) override;

private:
  // 常量（与原代码一致）
  static constexpr uint8_t WRITE_CMD = 0x02;
  static constexpr uint8_t RX_FRAME_LEN = 16;  // 原代码中反馈帧通常 >=12~16 字节，保守取 16

  // 底层写寄存器（len=3）
  bool write_register(uint8_t addr, uint8_t value);

  // 目标值转换（与原代码一致）
  int32_t get_target_pos(double pos_rad) const;
  int32_t get_target_curr(double effort) const;
};

}  // namespace single_arm_hardware_interface

#endif  // WHJ_MOTOR_DRIVER_HPP__