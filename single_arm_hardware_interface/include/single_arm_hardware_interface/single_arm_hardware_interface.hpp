// single_arm_hardware_interface.hpp
#ifndef SINGLE_ARM_HARDWARE_INTERFACE_HPP__
#define SINGLE_ARM_HARDWARE_INTERFACE_HPP__

#pragma once

#include <memory>
#include <vector>
#include <string>
#include <mutex>
#include <queue>
#include <unordered_map>
#include <unordered_set>
#include <atomic>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/macros.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "diagnostic_updater/diagnostic_updater.hpp"

#include "ros2_socketcan_msgs/msg/fd_frame.hpp"
#include "can_msgs/msg/frame.hpp"  // 添加 Frame include

#include "single_arm_hardware_interface/motor_driver_base.hpp"
#include "single_arm_hardware_interface/visibility_control.h"

using namespace std::chrono_literals;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using FdFrame = ros2_socketcan_msgs::msg::FdFrame;
using Frame = can_msgs::msg::Frame;

namespace single_arm_hardware_interface
{

class SingleArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  SingleArmHardwareInterface();
  virtual ~SingleArmHardwareInterface() = default;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  CAN_DRIVER_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  CAN_DRIVER_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  CAN_DRIVER_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) override;

  CAN_DRIVER_PUBLIC
  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces) override;

  CAN_DRIVER_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  CAN_DRIVER_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

  bool wait_for_subscriptions();

  bool is_configured() const;
  void set_configured(bool state);
  bool is_activated() const;
  void set_activated(bool state);

private:
  rclcpp::Logger logger_;
  std::shared_ptr<diagnostic_updater::Updater> updater_;
  rclcpp::CallbackGroup::SharedPtr can_rx_cbg_;  // 所有 CAN sub 共享一个 Reentrant group
  std::shared_ptr<rclcpp::Node> node_;

  std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;

  std::atomic<bool> shutdown_requested_;
  std::thread executor_thread_;

  std::queue<std::variant<FdFrame::SharedPtr, Frame::SharedPtr>> can_rx_buf_;
  std::mutex can_rx_buf_mutex_;

  std::unordered_map<uint8_t, size_t> can_id_to_index_;  // can_id -> joint index (假设全局唯一)

  std::string ns_;

  std::mutex mutex_;
  std::atomic<bool> activated_{false};
  std::atomic<bool> configured_{false};

  std::unordered_map<std::string, size_t> joint_indices_;

  std::vector<std::unique_ptr<MotorDriver>> drivers_;

  std::vector<double> hw_position_commands_;
  std::vector<double> hw_effort_commands_;

  std::vector<double> hw_position_states_;
  std::vector<double> hw_velocity_states_;
  std::vector<double> hw_effort_states_;
  std::vector<double> direction_multipliers_;
  std::vector<uint16_t> enable_states_;
  std::vector<uint16_t> error_states_;

  std::vector<bool> supports_position_command_;
  std::vector<bool> supports_effort_command_;

  std::mutex control_level_mutex_;
  std::vector<IntegrationLevel> control_level_;

  // 多 CAN 接口支持
  std::unordered_map<std::string, rclcpp::Publisher<FdFrame>::SharedPtr> fd_pubs_;
  std::unordered_map<std::string, rclcpp::Publisher<Frame>::SharedPtr> classic_pubs_;

  std::vector<rclcpp::Subscription<FdFrame>::SharedPtr> fd_subs_;
  std::vector<rclcpp::Subscription<Frame>::SharedPtr> classic_subs_;

  // 添加：保存 on_init 的 info
  hardware_interface::HardwareInfo hardware_info_;

  bool initialize_can_interfaces(const std::unordered_set<std::string>& unique_interfaces);
  void can_frame_cb_fd(const FdFrame::SharedPtr msg);
  void can_frame_cb_classic(const Frame::SharedPtr msg);
  void process_can_frame(const std::variant<FdFrame::SharedPtr, Frame::SharedPtr>& var_msg);

  void produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat);

  void executor_loop();

  constexpr static size_t MAX_QUEUE_SIZE = 4294967296;  // 类似 dual_arm
};

}  // namespace single_arm_hardware_interface

#endif  // SINGLE_ARM_HARDWARE_INTERFACE_HPP__