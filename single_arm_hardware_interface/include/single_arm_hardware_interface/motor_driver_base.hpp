// motor_driver_base.hpp
#ifndef MOTOR_DRIVER_BASE_HPP__
#define MOTOR_DRIVER_BASE_HPP__
#pragma once

#include <memory>
#include <vector>
#include <cstdint>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher_base.hpp"  // 添加 PublisherBase
#include "ros2_socketcan_msgs/msg/fd_frame.hpp"
#include "can_msgs/msg/frame.hpp"
#include "hardware_interface/hardware_info.hpp"

using FdFrame = ros2_socketcan_msgs::msg::FdFrame;
using Frame = can_msgs::msg::Frame;

// 与原代码保持一致的控制级别枚举
enum class IntegrationLevel : uint8_t
{
  UNDEFINED = 0,
  POSITION = 1,
  EFFORT = 2
};

namespace single_arm_hardware_interface
{

template <typename MsgType>
struct CanFrameTraits;

template <>
struct CanFrameTraits<FdFrame>
{
  static uint8_t get_len(const FdFrame& msg) { return msg.len; }
  static const uint8_t* get_data(const FdFrame& msg)
  {
    return msg.data.empty() ? nullptr : &msg.data[0];  // 修复：使用 &msg.data[0] 获取指针
  }
  static size_t get_max_len() { return 64; }
  static void set_len(FdFrame& msg, uint8_t len) { msg.len = len; }
  static void set_data(FdFrame& msg, const std::vector<uint8_t>& data)
  {
    msg.data.assign(data.begin(), data.end());
    if (msg.data.size() > 64) {
      msg.data.resize(64);  // 防止溢出
    }
  }
};

template <>
struct CanFrameTraits<Frame>
{
  static uint8_t get_len(const Frame& msg) { return msg.dlc; }
  static const uint8_t* get_data(const Frame& msg) { return msg.data.data(); }  // std::array<uint8_t, 8> 支持 data()
  static size_t get_max_len() { return 8; }
  static void set_len(Frame& msg, uint8_t len) { msg.dlc = len; }
  static void set_data(Frame& msg, const std::vector<uint8_t>& data)
  {
    if (data.size() > 8) {
      // 错误处理：经典 CAN 最多 8 字节
      throw std::length_error("Data too long for classic CAN frame");
    }
    std::copy(data.begin(), data.end(), msg.data.begin());
    std::fill(msg.data.begin() + data.size(), msg.data.end(), 0);  // 填充剩余为 0
  }
};

class MotorDriver
{
public:
  struct Config
  {
    uint8_t can_id = 0;
    int32_t position_offset = 0;
    std::string motor_type = "whj";
    std::string can_interface = "can1";
  };

  virtual ~MotorDriver() = default;

  virtual bool parse_parameters(const hardware_interface::ComponentInfo& joint) = 0;

  virtual bool send_iap() = 0;
  virtual bool send_clear_error() = 0;
  virtual bool send_set_mode(IntegrationLevel level) = 0;
  virtual bool send_enable(bool enable) = 0;
  virtual bool read_status() = 0;

  virtual void write_position_command(double position_cmd_rad) = 0;
  virtual void write_effort_command(double effort_cmd) = 0;

  // 两个 non-template virtual 函数（默认返回 false，子类覆盖）
  virtual bool process_fd_frame(const FdFrame::SharedPtr& msg,
                                double& out_position_rad,
                                double& out_velocity_rad_s,
                                double& out_effort,
                                uint16_t& out_enable_state,
                                uint16_t& out_error_code)
  {
    (void)msg;
    (void)out_position_rad;
    (void)out_velocity_rad_s;
    (void)out_effort;
    (void)out_enable_state;
    (void)out_error_code;
    return false;
  }

  virtual bool process_classic_frame(const Frame::SharedPtr& msg,
                                     double& out_position_rad,
                                     double& out_velocity_rad_s,
                                     double& out_effort,
                                     uint16_t& out_enable_state,
                                     uint16_t& out_error_code)
  {
    (void)msg;
    (void)out_position_rad;
    (void)out_velocity_rad_s;
    (void)out_effort;
    (void)out_enable_state;
    (void)out_error_code;
    return false;
  }

  uint8_t get_can_id() const { return config_.can_id; }

  void set_node(std::shared_ptr<rclcpp::Node> node) { node_weak_ = node; }
  void set_can_publisher(std::shared_ptr<rclcpp::PublisherBase> pub) { can_pub_weak_ = pub; }  // 统一 setter

protected:
  Config config_;

  std::weak_ptr<rclcpp::Node> node_weak_;
  std::weak_ptr<rclcpp::PublisherBase> can_pub_weak_;  // 统一为 PublisherBase

  rclcpp::Logger get_logger() const
  {
    if (auto node = node_weak_.lock()) {
      return node->get_logger();
    }
    return rclcpp::get_logger("MotorDriver");
  }

  template <typename MsgType>
  void send_frame(uint32_t id, const std::vector<uint8_t>& data)
  {
    auto pub_base = can_pub_weak_.lock();
    if (!pub_base) {
      RCLCPP_ERROR(get_logger(), "CAN publisher not available (id=0x%X)", id);
      return;
    }

    // dynamic_pointer_cast 到具体 Publisher<MsgType>
    auto pub = std::dynamic_pointer_cast<rclcpp::Publisher<MsgType>>(pub_base);
    if (!pub) {
      RCLCPP_ERROR(get_logger(), "Publisher type mismatch for MsgType (id=0x%X)", id);
      return;
    }

    MsgType frame;
    frame.id = id;

    uint8_t len = static_cast<uint8_t>(data.size());
    if (len > CanFrameTraits<MsgType>::get_max_len()) {
      RCLCPP_ERROR(get_logger(), "CAN data too long (%zu bytes > max %zu)", data.size(), CanFrameTraits<MsgType>::get_max_len());
      return;
    }

    CanFrameTraits<MsgType>::set_len(frame, len);
    CanFrameTraits<MsgType>::set_data(frame, data);

    if (auto node = node_weak_.lock()) {
      frame.header.stamp = node->now();
    } else {
      frame.header.stamp = rclcpp::Time(0);
    }

    // 设置通用标志
    if constexpr (std::is_same_v<MsgType, FdFrame>) {
      frame.is_extended = false;
      frame.is_error = false;
    } else {
      frame.is_rtr = false;
      frame.is_extended = false;
      frame.is_error = false;
    }

    pub->publish(frame);
  }

  template <typename MsgType>
  void send_frame(uint32_t id)
  {
    send_frame<MsgType>(id, {});
  }

  template <typename MsgType>
  void send_frame_uint32(uint32_t id, int32_t value)
  {
    std::vector<uint8_t> data(4);
    data[0] = static_cast<uint8_t>(value & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 8) & 0xFF);
    data[2] = static_cast<uint8_t>((value >> 16) & 0xFF);
    data[3] = static_cast<uint8_t>((value >> 24) & 0xFF);
    send_frame<MsgType>(id, data);
  }

  MotorDriver() = default;
};

}  // namespace single_arm_hardware_interface

#endif  // MOTOR_DRIVER_BASE_HPP__