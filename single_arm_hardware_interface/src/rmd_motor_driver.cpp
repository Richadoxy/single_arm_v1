// rmd_motor_driver.cpp
#include "single_arm_hardware_interface/rmd_motor_driver.hpp"

#include <cmath>

namespace single_arm_hardware_interface
{

bool RmdMotorDriver::parse_parameters(const hardware_interface::ComponentInfo& joint)
{
  try
  {
    // RMD 的 motor_id 通常是 0x141 ~ 0x148 等标准 CAN ID（11-bit）
    // 这里直接解析为 uint8_t（低 8 位即可，实际 CAN ID 使用低位）
    config_.can_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id"), nullptr, 16));

    // position_offset 可选（如果需要零点校准）
    if (joint.parameters.count("position_offset"))
    {
      config_.position_offset = static_cast<int32_t>(std::stol(joint.parameters.at("position_offset")));
    }
    else
    {
      config_.position_offset = 0;
    }

    if (joint.parameters.count("motor_type"))
    {
      config_.motor_type = joint.parameters.at("motor_type");
    }
    else
    {
      config_.motor_type = "rmd";  // 默认 rmd
    }
    if (joint.parameters.count("can_interface"))
    {
      config_.can_interface = joint.parameters.at("can_interface");
    }
    else
    {
      config_.can_interface = "can0";  // 默认 can0
    }
    RCLCPP_INFO(get_logger(), "RmdMotorDriver parsed: CAN ID=0x%02X, position_offset=%d, motor_type=%s, can_interface=%s",
                config_.can_id, config_.position_offset, config_.motor_type.c_str(), config_.can_interface.c_str());

    return true;
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Failed to parse parameters for joint %s: %s",
                 joint.name.c_str(), e.what());
    return false;
  }
}

void RmdMotorDriver::send_rmd_command(uint8_t cmd, const std::vector<uint8_t>& payload)
{
  std::vector<uint8_t> data(8, 0x00);
  
  if (!payload.empty())
  {
    std::copy(payload.begin(), payload.end(), data.begin());  // payload 从 data[4] 开始（常见 RMD 格式）
  }
  data[0] = cmd;
  uint32_t can_id = static_cast<uint32_t>(config_.can_id) + 0x140;  // 控制命令固定 +0x140
  send_frame<Frame>(can_id, data);
}

bool RmdMotorDriver::send_iap()
{
  // RMD 无 IAP 命令
  std::vector<uint8_t> payload(8);
  payload[0] = 0x00;
  payload[1] = 0x00;
  payload[2] = 0x00;
  payload[3] = 0x00;
  payload[4] = 0x00;
  payload[5] = 0x00;
  payload[6] = 0x00;
  payload[7] = 0x00;
  RCLCPP_WARN(get_logger(), "RMD [0x%02X] IAP command sent", config_.can_id);

  send_rmd_command(CMD_VELOCITY, payload);
  return true;
}

bool RmdMotorDriver::send_clear_error()
{
  // RMD 无明确清错命令（通常断电或发送正常命令清除）
  // 这里直接返回 true，后续可根据实际协议补充
  RCLCPP_WARN(get_logger(), "RMD [0x%02X] Clear error command sent", config_.can_id);
  return true;
}

bool RmdMotorDriver::send_set_mode(IntegrationLevel level)
{
  // RMD 无显式模式切换命令（通过发送对应控制命令自动切换）
  // 直接返回 true
  switch (level)
  {
    case IntegrationLevel::POSITION:
      send_enable(true);
      break;
    case IntegrationLevel::EFFORT:
      write_effort_command(0.0);
      break;
    default:
      RCLCPP_ERROR(get_logger(), "WHJ [0x%02X] Invalid control level for set_mode", config_.can_id);
      return false;
  }
  
  return true;
}

bool RmdMotorDriver::send_enable(bool enable)
{
  if (!enable)
  {
    // RMD 关闭电机（0x80 命令）
    write_effort_command(0.0);  // 先发送 0 努力
    RCLCPP_WARN(get_logger(), "RMD [0x%02X] Disable command sent", config_.can_id);
  } else
  {
    send_iap();
  }
  return true;
}

bool RmdMotorDriver::read_status()
{
  // RMD 通过发送读取状态命令获取状态
  send_enable(true);
  std::vector<uint8_t> payload(8);
  for (size_t i = 0; i < 8; ++i)
  {
    payload[i] = 0x00;
  }
  send_rmd_command(CMD_READ_STATUS, payload);
  RCLCPP_WARN(get_logger(), "RMD [0x%02X] Status request sent", config_.can_id);
  return true;
}
int32_t RmdMotorDriver::get_target_position_units(double pos_rad) const
{
  double pos_deg = pos_rad * 180.0 / M_PI;
  // RMD 多圈位置：0.01°/LSB
  return static_cast<int32_t>(pos_deg * 100.0) + config_.position_offset;
}

int16_t RmdMotorDriver::get_target_current_units(double effort_a) const
{
  // RMD 电流：0.01A/LSB（iq 控制）
  return static_cast<int16_t>(effort_a / 0.01);
}

void RmdMotorDriver::write_position_command(double position_cmd_rad)
{
  int32_t pos_units = get_target_position_units(position_cmd_rad);
  int16_t max_speed = 1000;
  std::vector<uint8_t> payload(8);
  payload[0] = 0x00;
  payload[1] = 0x00;
  payload[2] = static_cast<uint8_t>(max_speed & 0xFF);
  payload[3] = static_cast<uint8_t>((max_speed >> 8) & 0xFF);
  payload[4] = static_cast<uint8_t>(pos_units & 0xFF);
  payload[5] = static_cast<uint8_t>((pos_units >> 8) & 0xFF);
  payload[6] = static_cast<uint8_t>((pos_units >> 16) & 0xFF);
  payload[7] = static_cast<uint8_t>((pos_units >> 24) & 0xFF);
  // max_speed = 0 表示无速度限制
  // uint16_t max_speed = 0;

  // data[2:3] 为 max_speed（这里设 0）
  send_rmd_command(CMD_POSITION, payload);
}

void RmdMotorDriver::write_effort_command(double effort_cmd)
{
  int16_t cur_units = get_target_current_units(effort_cmd);

  std::vector<uint8_t> payload(8);
  payload[0] = 0x00;
  payload[1] = 0x00;
  payload[2] = 0x00;
  payload[3] = 0x00;
  payload[4] = static_cast<uint8_t>(cur_units & 0xFF);
  payload[5] = static_cast<uint8_t>((cur_units >> 8) & 0xFF);
  payload[6] = 0x00;
  payload[7] = 0x00;

  send_rmd_command(CMD_TORQUE, payload);
}

bool RmdMotorDriver::process_classic_frame(const Frame::SharedPtr& msg,  
                                           double& out_position_rad,
                                           double& out_velocity_rad_s,
                                           double& out_effort,
                                           uint16_t& out_enable_state,
                                           uint16_t& out_error_code)
{
  uint32_t expected_feedback_id = static_cast<uint32_t>(config_.can_id) + 0x240;

  if (msg->id != expected_feedback_id || msg->dlc < 8)
  {
    return false;  // 不属于本驱动
  }

  // uint8_t len = CanFrameTraits<Frame>::get_len(*msg);
  const uint8_t* data = CanFrameTraits<Frame>::get_data(*msg);

  uint8_t cmd = data[0];

  if (cmd == 0x9A || cmd == 0xA1 || cmd == 0xA2 || cmd == 0xA4)  
  {
    // 温度 (data[1]) 忽略
    int16_t iq      = static_cast<int16_t>((data[3] << 8) | data[2]);
    int16_t speed   = static_cast<int16_t>((data[5] << 8) | data[4]);
    int16_t angle   = static_cast<int16_t>((data[7] << 8) | data[6]);  // 多圈角度（0.01°/LSB）
    // RCLCPP_INFO(get_logger(), "RMD [0x%02X] Feedback: data[7]: %02X, data[6]: %02X",
    //              config_.can_id, data[7], data[6]);
    // RCLCPP_INFO(get_logger(), "RMD [0x%02X] Feedback: cmd=0x%02X, iq=%d, speed=%d, angle=%d",
    //              config_.can_id, cmd, iq, speed, angle);
    out_effort = iq * 0.01;                                            // A
    double vel_rpm = speed / 6.0;                                      // dps -> rpm（原代码逻辑）
    out_velocity_rad_s = vel_rpm * (2.0 * M_PI / 60.0);                 // rpm -> rad/s
    out_position_rad = (angle) * (M_PI / 180.0);                 // 0.01° -> deg -> rad

    out_enable_state = 1;  // RMD 发送控制命令后默认启用
    out_error_code = 0;

    return true;
  }
  else if (cmd == 0x92)  // 仅位置反馈（多圈）
  {
    int32_t angle = static_cast<int32_t>((data[7] << 24) | (data[6] << 16) |
                                         (data[5] << 8)  | data[4]);

    out_position_rad = (angle * 0.01) * (M_PI / 180.0);
    out_enable_state = 1;
    out_error_code = 0;

    return true;
  }
  else  // 错误反馈或其他
  {
    int16_t error = static_cast<int16_t>((data[7] << 8) | data[6]);
    out_error_code = static_cast<uint16_t>(error);

    if (out_error_code != 0)
    {
      RCLCPP_ERROR(get_logger(), "RMD [0x%02X] Error code: 0x%04X", config_.can_id, out_error_code);
    }

    return true;
  }

  return false;
}

}  // namespace single_arm_hardware_interface