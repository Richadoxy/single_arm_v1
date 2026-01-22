// whj_motor_driver.cpp
#include "single_arm_hardware_interface/whj_motor_driver.hpp"

#include <cmath>
#include <limits>

namespace single_arm_hardware_interface
{

bool WHJMotorDriver::parse_parameters(const hardware_interface::ComponentInfo& joint)
{
  try
  {
    // 解析 can_id（16进制）
    config_.can_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id"), nullptr, 16));

    // 解析 position_offset（可选，默认为 0）
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
      config_.motor_type = "whj";  // 默认 whj
    }
    if (joint.parameters.count("can_interface"))
    {
      config_.can_interface = joint.parameters.at("can_interface");
    }
    else
    {
      config_.can_interface = "can1";  // 默认 can1
    }
    RCLCPP_INFO(get_logger(), "WHJMotorDriver parsed: CAN ID=0x%02X, position_offset=%d, motor_type=%s, can_interface=%s",
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

bool WHJMotorDriver::write_register(uint8_t addr, uint8_t value)
{
  std::vector<uint8_t> data = {WRITE_CMD, addr, value};
  send_frame<FdFrame>(config_.can_id, data);
  return true;  // publish 总是成功（ROS2 publisher 无返回值）
}

bool WHJMotorDriver::send_iap()
{
  bool success = write_register(MotorAddr::IAP_FLAG, 0x00);
  if (success)
  {
    RCLCPP_WARN(get_logger(), "WHJ [0x%02X] IAP command sent", config_.can_id);
  }
  return success;
}

bool WHJMotorDriver::send_clear_error()
{
  bool success = write_register(MotorAddr::CLEAR_ERROR, 0x01);
  if (success)
  {
    RCLCPP_WARN(get_logger(), "WHJ [0x%02X] Clear error command sent", config_.can_id);
  }
  return success;
}

bool WHJMotorDriver::send_set_mode(IntegrationLevel level)
{
  uint8_t mode = 0;
  switch (level)
  {
    case IntegrationLevel::POSITION:
      mode = MotorMode::POSITION_MODE;
      break;
    case IntegrationLevel::EFFORT:
      mode = MotorMode::EFFORT_MODE;
      break;
    default:
      RCLCPP_ERROR(get_logger(), "WHJ [0x%02X] Invalid control level for set_mode", config_.can_id);
      return false;
  }

  bool success = write_register(MotorAddr::WORK_MODE, mode);
  if (success)
  {
    RCLCPP_WARN(get_logger(), "WHJ [0x%02X] Set mode to %d", config_.can_id, mode);
  }
  return success;
}

bool WHJMotorDriver::send_enable(bool enable)
{
  uint8_t flag = enable ? 0x01 : 0x00;
  bool success = write_register(MotorAddr::ENABLE_FLAG, flag);
  if (success)
  {
    RCLCPP_WARN(get_logger(), "WHJ [0x%02X] %s command sent",
                config_.can_id, enable ? "Enable" : "Disable");
  }
  return success;
}

bool WHJMotorDriver::read_status()
{
  send_frame<FdFrame>(config_.can_id + CanIdOffset::STATUS_REQ_ID_OFFSET);
  RCLCPP_WARN(get_logger(), "WHJ [0x%02X] Status request sent", config_.can_id);
  return true;
}
int32_t WHJMotorDriver::get_target_pos(double pos_rad) const
{
  // rad → deg → 0.0001°单位 + offset
  return static_cast<int32_t>(pos_rad / M_PI * 180.0 * 10000.0) + config_.position_offset;
}

int32_t WHJMotorDriver::get_target_curr(double effort) const
{
  // A → mA
  return static_cast<int32_t>(effort * 1000.0);
}

void WHJMotorDriver::write_position_command(double position_cmd_rad)
{
  int32_t target_pos = get_target_pos(position_cmd_rad);
  send_frame_uint32<FdFrame>(config_.can_id + CanIdOffset::POS_CTRL_ID_OFFSET, target_pos);
}

void WHJMotorDriver::write_effort_command(double effort_cmd)
{
  int32_t target_curr = get_target_curr(effort_cmd);
  send_frame_uint32<FdFrame>(config_.can_id + CanIdOffset::CUR_CTRL_ID_OFFSET, target_curr);
  auto node = node_weak_.lock();
  auto clock = node->get_clock();
  // 原代码中的提示（可保留）
  RCLCPP_WARN_THROTTLE(get_logger(), *clock, 5000,
                       "WHJ [0x%02X] Effort mode command sent (target_curr=%d)", config_.can_id, target_curr);
}
bool WHJMotorDriver::process_fd_frame(const FdFrame::SharedPtr& msg,  
                                      double& out_position_rad,
                                      double& out_velocity_rad_s,
                                      double& out_effort,
                                      uint16_t& out_enable_state,
                                      uint16_t& out_error_code)
{
  uint32_t expected_servo_id = config_.can_id + CanIdOffset::SERVO_RESP_ID_OFFSET;
  uint32_t expected_status_id = config_.can_id + CanIdOffset::STATUS_RESP_ID_OFFSET;

  if (msg->id != expected_servo_id && msg->id != expected_status_id)
  {
    return false;  // 不属于本驱动
  }
  auto node = node_weak_.lock();
  if (!node) {
    RCLCPP_ERROR(get_logger(), "Node not available in process_frame");
    return false;
  }
  auto clock = node->get_clock();
  if (msg->len < RX_FRAME_LEN)
  {
    RCLCPP_ERROR_THROTTLE(get_logger(), *clock, 1000,
                      "WHJ [0x%02X] Invalid frame length %d (expected >= %d)",
                      config_.can_id, msg->len, RX_FRAME_LEN);
    return false;
  }
  // uint8_t len = CanFrameTraits<FdFrame>::get_len(*msg);
  // const uint8_t* data = CanFrameTraits<FdFrame>::get_data(*msg);
  // 提取工具 lambda
  auto extract_int32 = [&](std::size_t offset) -> int32_t {
    return static_cast<int32_t>(
      (msg->data[offset + 3] << 24) |
      (msg->data[offset + 2] << 16) |
      (msg->data[offset + 1] << 8)  |
      msg->data[offset]
    );
  };

  auto extract_uint16 = [&](std::size_t offset) -> uint16_t {
    return static_cast<uint16_t>(
      (msg->data[offset + 1] << 8) |
      msg->data[offset]
    );
  };

  double raw_pos_deg = extract_int32(8) * 0.0001 - config_.position_offset * 0.0001;

  if (msg->id == expected_servo_id)  // SERVO_RESP（实时反馈）
  {
    double vel_raw = extract_int32(4) * 0.02;          // 单位未知，原始代码如此
    double curr_ma = extract_int32(0);                // mA
    out_enable_state = extract_uint16(12);
    out_error_code = extract_uint16(14);

    out_position_rad = raw_pos_deg / 180.0 * M_PI;
    out_velocity_rad_s = vel_raw / 30.0 * M_PI;        // 原始代码转换（疑似 RPM → rad/s）
    out_effort = curr_ma * 0.001;                      // mA → A

    if (out_error_code != 0)
    {
      RCLCPP_ERROR(get_logger(), "WHJ [0x%02X] Error 0x%04X: %s",
                   config_.can_id, out_error_code, error_code_to_str(out_error_code).c_str());
    }

    return true;
  }
  else  // STATUS_RESP（状态查询响应，带多圈连续处理）
  {
    if (raw_pos_deg > 180.0)
    {
      raw_pos_deg -= 360.0;
      config_.position_offset += 3600000;
      RCLCPP_INFO(get_logger(), "WHJ [0x%02X] Multi-turn +360°, new offset=%d",
                  config_.can_id, config_.position_offset);
    }
    else if (raw_pos_deg < -180.0)
    {
      raw_pos_deg += 360.0;
      config_.position_offset -= 3600000;
      RCLCPP_INFO(get_logger(), "WHJ [0x%02X] Multi-turn -360°, new offset=%d",
                  config_.can_id, config_.position_offset);
    }

    out_position_rad = raw_pos_deg / 180.0 * M_PI;
    // 其他状态未知，填 0（主接口只需初始位置）
    out_velocity_rad_s = 0.0;
    out_effort = 0.0;
    out_enable_state = 0;
    out_error_code = 0;

    return true;
  }
}

}  // namespace single_arm_hardware_interface