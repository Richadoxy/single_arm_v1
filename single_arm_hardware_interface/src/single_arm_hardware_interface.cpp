// single_arm_hardware_interface.cpp
#include "single_arm_hardware_interface/single_arm_hardware_interface.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <mutex>
#include <thread>
#include <unordered_set>
#include <variant>
#include "rclcpp/executors.hpp"
#include "rclcpp/utilities.hpp"

#include "single_arm_hardware_interface/whj_motor_driver.hpp"
#include "single_arm_hardware_interface/rmd_motor_driver.hpp"

namespace single_arm_hardware_interface
{

SingleArmHardwareInterface::SingleArmHardwareInterface()
  : hardware_interface::SystemInterface(),
    logger_(rclcpp::get_logger("single_arm_hardware_interface")),
    executor_(std::make_shared<rclcpp::executors::MultiThreadedExecutor>())
{
  shutdown_requested_.store(false);
  executor_thread_ = std::thread(std::bind(&SingleArmHardwareInterface::executor_loop, this));

  RCLCPP_INFO(logger_, "SingleArmHardwareInterface initialized");
}

CallbackReturn SingleArmHardwareInterface::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;
  hardware_info_ = info;
  std::lock_guard<std::mutex> lock(mutex_);
  joint_indices_.reserve(info.joints.size());
  supports_position_command_.resize(info.joints.size(), false);
  supports_effort_command_.resize(info.joints.size(), false);

  drivers_.reserve(info.joints.size());

  std::unordered_set<std::string> unique_can_interfaces;

  for (size_t i = 0; i < info.joints.size(); ++i)
  {
    const auto& joint = info.joints[i];
    joint_indices_[joint.name] = i;

    // 根据 motor_type 创建对应的驱动实例
    std::unique_ptr<MotorDriver> driver;
    std::string motor_type = joint.parameters.count("motor_type") ? joint.parameters.at("motor_type") : "whj";
    if (motor_type == "whj")
    {
      driver = std::make_unique<WHJMotorDriver>();
    }
    else if (motor_type == "rmd")
    {
      driver = std::make_unique<RmdMotorDriver>();
    }
    else
    {
      RCLCPP_ERROR(logger_, "Unsupported motor_type '%s' for joint %s", motor_type.c_str(), joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    // 解析参数（驱动内部处理 can_id, position_offset 等）
    if (!driver->parse_parameters(joint))
    {
      RCLCPP_ERROR(logger_, "Failed to parse parameters for joint %s", joint.name.c_str());
      return CallbackReturn::ERROR;
    }

    // 收集唯一的 can_interface
    std::string can_interface = joint.parameters.count("can_interface") ? joint.parameters.at("can_interface") : "can1";
    unique_can_interfaces.insert(can_interface);

    // 检查命令接口支持
    for (const auto& cmd_if : joint.command_interfaces)
    {
      if (cmd_if.name == hardware_interface::HW_IF_POSITION)
      {
        supports_position_command_[i] = true;
      }
      else if (cmd_if.name == hardware_interface::HW_IF_EFFORT)
      {
        supports_effort_command_[i] = true;
      }
    }

    drivers_.push_back(std::move(driver));

    RCLCPP_INFO(logger_, "Initialized joint %s (CAN ID: 0x%X, motor_type: %s, can_interface: %s)",
                joint.name.c_str(), drivers_[i]->get_can_id(), motor_type.c_str(), can_interface.c_str());
  }

  // 构建 can_id_to_index_（假设 can_id 全局唯一）
  can_id_to_index_.clear();
  for (size_t i = 0; i < drivers_.size(); ++i)
  {
    uint8_t can_id = drivers_[i]->get_can_id();
    if (can_id_to_index_.count(can_id))
    {
      RCLCPP_ERROR(logger_, "Duplicate CAN ID 0x%X detected", can_id);
      return CallbackReturn::ERROR;
    }
    can_id_to_index_[can_id] = i;
    RCLCPP_DEBUG(logger_, "CAN-ID [0x%X] mapped to index [%zu]", can_id, i);
  }

  // 初始化状态和命令数组
  hw_position_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_effort_states_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());

  enable_states_.resize(info.joints.size(), std::numeric_limits<uint16_t>::quiet_NaN());
  error_states_.resize(info.joints.size(), std::numeric_limits<uint16_t>::quiet_NaN());

  hw_position_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_effort_commands_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  direction_multipliers_.resize(info.joints.size(), std::numeric_limits<double>::quiet_NaN());
  control_level_.resize(info.joints.size(), IntegrationLevel::POSITION);

  // 从 hardware_parameters 获取 namespace（如果有）
  if (info.hardware_parameters.count("namespace"))
  {
    ns_ = info.hardware_parameters.at("namespace");
  }
  else
  {
    ns_ = "";
    RCLCPP_WARN(logger_, "No namespace provided, using empty namespace");
  }

  // 创建节点
  node_ = std::make_shared<rclcpp::Node>("single_arm_hw_interface_" + info.name);

  // 初始化 CAN 接口（多个）
  if (!initialize_can_interfaces(unique_can_interfaces))
  {
    RCLCPP_ERROR(logger_, "Failed to initialize CAN interfaces");
    return CallbackReturn::ERROR;
  }

  // 为每个驱动设置 node 和 publisher（根据其 can_interface）
  for (size_t i = 0; i < drivers_.size(); ++i)
  {
    drivers_[i]->set_node(node_);
    std::string can_interface = info.joints[i].parameters.count("can_interface") ? info.joints[i].parameters.at("can_interface") : "can1";
    bool is_fd = (can_interface == "can1");  // 假设 can1 是 FD, can0 是经典

    // 修复类型不匹配：显式转换为 PublisherBase
    std::shared_ptr<rclcpp::PublisherBase> pub;
    if (is_fd) {
      pub = std::static_pointer_cast<rclcpp::PublisherBase>(fd_pubs_[can_interface]);
    } else {
      pub = std::static_pointer_cast<rclcpp::PublisherBase>(classic_pubs_[can_interface]);
    }
    drivers_[i]->set_can_publisher(pub);  // 统一调用
  }

  // 初始化 diagnostic updater
  if (node_)
  {
    updater_ = std::make_shared<diagnostic_updater::Updater>(node_);
    updater_->setHardwareID(info.name);

    updater_->add(info.name + "_Status", this, &SingleArmHardwareInterface::produce_diagnostics);
    RCLCPP_INFO(logger_, "Added diagnostics to node");
  }
  else
  {
    RCLCPP_WARN(logger_, "Node not available. Diagnostics will not be published");
  }
  direction_multipliers_[0] = 1.0;  // 关节2
  direction_multipliers_[1] = -1.0;  // 关节4
  direction_multipliers_[2] = 1.0;  // 关节6
  direction_multipliers_[3] = -1.0;  // 关节2
  direction_multipliers_[4] = 1.0;  // 关节4
  direction_multipliers_[5] = -1.0;  // 关节6
  executor_->add_node(node_);
  
  RCLCPP_INFO(logger_, "Added node to the executor");

  RCLCPP_INFO(logger_, "Successfully initialized %zu motors", drivers_.size());
  return CallbackReturn::SUCCESS;
}

bool SingleArmHardwareInterface::initialize_can_interfaces(const std::unordered_set<std::string>& unique_interfaces)
{
  can_rx_cbg_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  for (const auto& iface : unique_interfaces)
  {
    std::string to_topic, from_topic;
    bool is_fd = (iface == "can1");  // 假设 can1 是 FD, can0 是经典

    if (is_fd) {
      to_topic = "/to_can_bus_fd";
      from_topic = "/from_can_bus_fd";
    } else {
      to_topic = "/to_can_bus";
      from_topic = "/from_can_bus";
    }

    rclcpp::SubscriptionOptions sub_options;
    sub_options.callback_group = can_rx_cbg_;

    if (is_fd) {
      fd_pubs_[iface] = node_->create_publisher<FdFrame>(to_topic, rclcpp::QoS(10).reliable());

      auto sub = node_->create_subscription<FdFrame>(
        from_topic,
        rclcpp::QoS(10).reliable(),
        std::bind(&SingleArmHardwareInterface::can_frame_cb_fd, this, std::placeholders::_1),
        sub_options);
      fd_subs_.push_back(sub);
    } else {
      classic_pubs_[iface] = node_->create_publisher<Frame>(to_topic, rclcpp::QoS(10).reliable());

      auto sub = node_->create_subscription<Frame>(
        from_topic,
        rclcpp::QoS(10).reliable(),
        std::bind(&SingleArmHardwareInterface::can_frame_cb_classic, this, std::placeholders::_1),
        sub_options);
      classic_subs_.push_back(sub);
    }
  }

  return true;
}

CallbackReturn SingleArmHardwareInterface::on_configure(const rclcpp_lifecycle::State& /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  for (size_t i = 0; i < drivers_.size(); ++i)
  {
    auto& driver = drivers_[i];
    if (driver->get_can_id() == 0)
    {
      RCLCPP_ERROR(logger_, "Invalid CAN ID for motor %zu", i);
      return CallbackReturn::ERROR;
    }

    if (!driver->send_iap())
    {
      RCLCPP_ERROR(logger_, "Failed to send IAP to motor ID: 0x%X", driver->get_can_id());
      return CallbackReturn::ERROR;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if (!driver->send_clear_error())
    {
      RCLCPP_WARN(logger_, "Failed to clear errors on motor ID: 0x%X", driver->get_can_id());
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    switch (control_level_[i])
    {
      case IntegrationLevel::UNDEFINED:
        RCLCPP_ERROR(logger_, "UNDEFINED control level for motor ID: 0x%X", driver->get_can_id());
        break;
      case IntegrationLevel::POSITION:
        if (!driver->send_set_mode(IntegrationLevel::POSITION))
        {
          RCLCPP_ERROR(logger_, "Failed to set position mode for motor ID: 0x%X", driver->get_can_id());
          return CallbackReturn::ERROR;
        }
        break;
      case IntegrationLevel::EFFORT:
        if (!driver->send_set_mode(IntegrationLevel::EFFORT))
        {
          RCLCPP_ERROR(logger_, "Failed to set effort mode for motor ID: 0x%X", driver->get_can_id());
          return CallbackReturn::ERROR;
        }
        break;
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(200));
  }
  set_configured(true);
  RCLCPP_INFO(logger_, "Successfully configured %zu motors", drivers_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SingleArmHardwareInterface::on_activate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  RCLCPP_INFO(logger_, "Starting activate state");

  if (is_activated())
  {
    RCLCPP_FATAL(logger_, "Double on_activate()");
    return CallbackReturn::ERROR;
  }

  for (size_t i = 0; i < drivers_.size(); ++i)
  {
    // 统一调用驱动的 read_status(true) 来触发初始状态反馈
    // 具体行为由 WHJ/RMD 各自实现（WHJ 发送 STATUS_REQ，RMD 发送 enable(true)）
    if (!drivers_[i]->read_status())
    {
      RCLCPP_ERROR(logger_, "Failed to trigger initial status read for joint %zu (motor_type: %s, CAN ID: 0x%X)",
                   i, info_.joints[i].parameters.at("motor_type").c_str(), drivers_[i]->get_can_id());
      return CallbackReturn::ERROR;
    }

    // 等待初始位置反馈（超时 10s，和 dual_arm 完全一致）
    const auto start_time = std::chrono::steady_clock::now();
    const auto timeout = std::chrono::seconds(10);
    bool position_received = false;

    while (rclcpp::ok() && std::chrono::steady_clock::now() - start_time < timeout)
    {
      {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!std::isnan(hw_position_states_[i]))
        {
          RCLCPP_INFO(logger_, "Initial position received for joint %zu (%s): %f",
                      i, info_.joints[i].name.c_str(), hw_position_states_[i]);
          position_received = true;
          break;
        }
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    if (!position_received)
    {
      RCLCPP_ERROR(logger_, "Timeout waiting for initial position from joint %zu (motor_type: %s, CAN ID: 0x%X)",
                   i, info_.joints[i].parameters.at("motor_type").c_str(), drivers_[i]->get_can_id());
      return CallbackReturn::ERROR;
    }
  }

  set_activated(true);

  RCLCPP_INFO(logger_, "Successfully activated %zu motors", drivers_.size());
  return CallbackReturn::SUCCESS;
}

CallbackReturn SingleArmHardwareInterface::on_deactivate(const rclcpp_lifecycle::State& /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  set_activated(false);

  for (const auto& driver : drivers_)
  {
    driver->send_enable(false);
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn SingleArmHardwareInterface::on_cleanup(const rclcpp_lifecycle::State& /*previous_state*/)
{
  std::lock_guard<std::mutex> lock(mutex_);
  set_configured(false);

  return CallbackReturn::SUCCESS;
}

CallbackReturn SingleArmHardwareInterface::on_shutdown(const rclcpp_lifecycle::State& /*previous_state*/)
{
  shutdown_requested_.store(true);
  if (executor_thread_.joinable())
  {
    executor_thread_.join();
  }

  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> SingleArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < hardware_info_.joints.size(); ++i)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      hardware_info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      hardware_info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocity_states_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      hardware_info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_states_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> SingleArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < hardware_info_.joints.size(); ++i)
  {
    if (supports_position_command_[i])
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        hardware_info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_position_commands_[i]));
    }
    if (supports_effort_command_[i])
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        hardware_info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_effort_commands_[i]));
    }
  }

  return command_interfaces;
}

hardware_interface::return_type SingleArmHardwareInterface::prepare_command_mode_switch(
    const std::vector<std::string>& start_interfaces,
    const std::vector<std::string>& stop_interfaces)
{
  std::lock_guard<std::mutex> control_lock(control_level_mutex_);

  // Step 1: 先处理 stop_interfaces：清空旧模式（设为 UNDEFINED）
  for (const auto& key : stop_interfaces)
  {
    for (size_t i = 0; i < hardware_info_.joints.size(); ++i)
    {
      std::string position_if = hardware_info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
      std::string effort_if   = hardware_info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;

      if (key == position_if || key == effort_if)
      {
        control_level_[i] = IntegrationLevel::UNDEFINED;
        RCLCPP_INFO(logger_, "Prepare: joint %s old interface stopped, set to UNDEFINED",
                    hardware_info_.joints[i].name.c_str());
      }
    }
  }

  // Step 2: 再处理 start_interfaces：设置新模式
  for (const auto& key : start_interfaces)
  {
    for (size_t i = 0; i < hardware_info_.joints.size(); ++i)
    {
      std::string position_if = hardware_info_.joints[i].name + "/" + hardware_interface::HW_IF_POSITION;
      std::string effort_if   = hardware_info_.joints[i].name + "/" + hardware_interface::HW_IF_EFFORT;

      if (key == position_if)
      {
        control_level_[i] = IntegrationLevel::POSITION;
        RCLCPP_INFO(logger_, "Prepare: joint %s set to POSITION mode",
                    hardware_info_.joints[i].name.c_str());
      }
      else if (key == effort_if)
      {
        control_level_[i] = IntegrationLevel::EFFORT;
        RCLCPP_INFO(logger_, "Prepare: joint %s set to EFFORT mode",
                    hardware_info_.joints[i].name.c_str());
      }
    }
  }

  // Step 3: 可选防御性检查（防止遗漏设置）
  for (size_t i = 0; i < hardware_info_.joints.size(); ++i)
  {
    if (control_level_[i] == IntegrationLevel::UNDEFINED && !start_interfaces.empty())
    {
      RCLCPP_ERROR(logger_, "Prepare failed: joint %s remained UNDEFINED after switch",
                   hardware_info_.joints[i].name.c_str());
      return hardware_interface::return_type::ERROR;
    }
  }

  return hardware_interface::return_type::OK;
}
hardware_interface::return_type SingleArmHardwareInterface::perform_command_mode_switch(
    const std::vector<std::string>& /*start_interfaces*/,
    const std::vector<std::string>& /*stop_interfaces*/)
{
  std::lock_guard<std::mutex> lock(control_level_mutex_);

  for (size_t i = 0; i < drivers_.size(); ++i)
  {
    IntegrationLevel level = control_level_[i];

    bool success = false;
    switch (level)
    {
      case IntegrationLevel::POSITION:
        if (!std::isnan(hw_position_states_[i])) {
          hw_position_commands_[i] = hw_position_states_[i];
          RCLCPP_INFO(logger_, "Joint %zu (CAN ID 0x%02X) POSITION mode: Set initial command to current state %.4f",
                      i, drivers_[i]->get_can_id(), hw_position_states_[i]);
        } else {
          RCLCPP_WARN(logger_, "Joint %zu: Current position state is NaN, skipping initial command set", i);
        }
        success = drivers_[i]->send_set_mode(IntegrationLevel::POSITION);
        RCLCPP_INFO(logger_, "Joint %zu (CAN ID 0x%02X) switched to POSITION mode",
                    i, drivers_[i]->get_can_id());
        break;

      case IntegrationLevel::EFFORT:
        success = drivers_[i]->send_set_mode(IntegrationLevel::EFFORT);
        RCLCPP_INFO(logger_, "Joint %zu (CAN ID 0x%02X) switched to EFFORT mode",
                    i, drivers_[i]->get_can_id());
        break;

      case IntegrationLevel::UNDEFINED:
      default:
        RCLCPP_ERROR(logger_, "Joint %zu has UNDEFINED control level during perform switch!", i);
        return hardware_interface::return_type::ERROR;
    }

    if (!success)
    {
      RCLCPP_ERROR(logger_, "Failed to send mode command to joint %zu (CAN ID 0x%02X)", i, drivers_[i]->get_can_id());
      return hardware_interface::return_type::ERROR;
    }
  }

  RCLCPP_INFO(logger_, "Perform command mode switch completed successfully");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SingleArmHardwareInterface::read(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  std::queue<std::variant<FdFrame::SharedPtr, Frame::SharedPtr>> temp_queue;
  {
    std::lock_guard<std::mutex> lock(can_rx_buf_mutex_);
    temp_queue.swap(can_rx_buf_);
  }

  while (!temp_queue.empty())
  {
    auto var_msg = temp_queue.front();
    temp_queue.pop();

    try
    {
      process_can_frame(var_msg);
    }
    catch (const std::exception& e)
    {
      RCLCPP_ERROR(logger_, "Error processing CAN frame: %s", e.what());
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SingleArmHardwareInterface::write(const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_activated())
  {
    return hardware_interface::return_type::OK;
  }

  std::lock_guard<std::mutex> control_lock(control_level_mutex_);
  for (size_t i = 0; i < drivers_.size(); ++i)
  {
    switch (control_level_[i])
    {
      case IntegrationLevel::POSITION:
        if (!std::isnan(hw_position_commands_[i]))
        {
          double cmd = hw_position_commands_[i] * direction_multipliers_[i];
          drivers_[i]->write_position_command(cmd);
        }
        break;
      case IntegrationLevel::EFFORT:
      {
        double effort_cmd = hw_effort_commands_[i]* direction_multipliers_[i];
        if (std::isnan(effort_cmd))
        {
          effort_cmd = 0.0;  // 默认 0 电流，保持松弛但保活
        }
        drivers_[i]->write_effort_command(effort_cmd);
      }
      break;
      default:
        break;
    }
  }

  return hardware_interface::return_type::OK;
}

void SingleArmHardwareInterface::can_frame_cb_fd(const FdFrame::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(can_rx_buf_mutex_);

    if (can_rx_buf_.size() < MAX_QUEUE_SIZE)
    {
      can_rx_buf_.push(msg);
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(logger_, *(node_->get_clock()), 1000, "CAN RX queue full, dropping frame");
    }
  }

  // 直接处理一份（类似 dual_arm）
  process_can_frame(std::variant<FdFrame::SharedPtr, Frame::SharedPtr>(msg));
}

void SingleArmHardwareInterface::can_frame_cb_classic(const Frame::SharedPtr msg)
{
  {
    std::lock_guard<std::mutex> lock(can_rx_buf_mutex_);

    if (can_rx_buf_.size() < MAX_QUEUE_SIZE)
    {
      can_rx_buf_.push(msg);
    }
    else
    {
      RCLCPP_ERROR_THROTTLE(logger_, *(node_->get_clock()), 1000, "CAN RX queue full, dropping frame");
    }
  }

  // 直接处理一份（类似 dual_arm）
  process_can_frame(std::variant<FdFrame::SharedPtr, Frame::SharedPtr>(msg));
}

void SingleArmHardwareInterface::process_can_frame(const std::variant<FdFrame::SharedPtr, Frame::SharedPtr>& var_msg)
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (!is_configured())
  {
    return;
  }

  if (drivers_.empty() || hw_position_states_.empty())
  {
    RCLCPP_ERROR(logger_, "Hardware interface not fully initialized");
    return;
  }

  uint32_t msg_id;
  std::visit([&](auto&& msg) {
    msg_id = msg->id;
  }, var_msg);

  // 提取 base_can_id（类似 dual_arm 的 0xF mask）
  const uint8_t base_can_id = msg_id & 0xF;
  RCLCPP_DEBUG(logger_, "Received CAN ID: 0x%X (base: 0x%X)", msg_id, base_can_id);

  auto map_it = can_id_to_index_.find(base_can_id);
  if (map_it == can_id_to_index_.end())
  {
    RCLCPP_DEBUG(logger_, "Unknown base CAN ID: 0x%X", base_can_id);
    return;
  }

  const size_t joint_index = map_it->second;

  double pos_rad = 0.0;
  double vel_rad_s = 0.0;
  double effort = 0.0;
  uint16_t enable_state = 0;
  uint16_t error_code = 0;

  bool processed = false;
  if (std::holds_alternative<FdFrame::SharedPtr>(var_msg)) {
    auto msg = std::get<FdFrame::SharedPtr>(var_msg);
    processed = drivers_[joint_index]->process_fd_frame(msg, pos_rad, vel_rad_s, effort, enable_state, error_code);
  } else if (std::holds_alternative<Frame::SharedPtr>(var_msg)) {
    auto msg = std::get<Frame::SharedPtr>(var_msg);
    processed = drivers_[joint_index]->process_classic_frame(msg, pos_rad, vel_rad_s, effort, enable_state, error_code);
  }

  if (processed)
  {
    double sign = direction_multipliers_[joint_index];
    hw_position_states_[joint_index] = pos_rad* sign;
    hw_velocity_states_[joint_index] = vel_rad_s* sign;
    hw_effort_states_[joint_index] = effort* sign;
    enable_states_[joint_index] = enable_state;
    error_states_[joint_index] = error_code;
  }
  else
  {
    RCLCPP_DEBUG(logger_, "Frame not processed by driver for joint %zu", joint_index);
  }
}

void SingleArmHardwareInterface::produce_diagnostics(diagnostic_updater::DiagnosticStatusWrapper& stat)
{
  stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Hardware is OK");
}

void SingleArmHardwareInterface::executor_loop()
{
  RCLCPP_INFO(logger_, "Starting executor loop");

  while (rclcpp::ok() && !shutdown_requested_.load())
  {
    executor_->spin_once();
  }

  RCLCPP_INFO(logger_, "Ending executor loop");
}

bool SingleArmHardwareInterface::wait_for_subscriptions()
{
  const uint8_t MAX_ATTEMPTS = 60;
  uint8_t attempts = 0;
  rclcpp::Rate rate(1);

  while (rclcpp::ok())
  {
    bool all_ready = true;
    for (const auto& [iface, pub] : fd_pubs_)
    {
      if (pub->get_subscription_count() == 0)
      {
        all_ready = false;
        break;
      }
    }
    for (const auto& [iface, pub] : classic_pubs_)
    {
      if (pub->get_subscription_count() == 0)
      {
        all_ready = false;
        break;
      }
    }

    if (all_ready)
      break;

    if (attempts > MAX_ATTEMPTS)
      return false;

    attempts++;
    RCLCPP_WARN(logger_, "Waiting for socketcan subscriptions (%d/%d)", attempts, MAX_ATTEMPTS);
    rate.sleep();
  }

  return true;
}

bool SingleArmHardwareInterface::is_configured() const
{
  return configured_.load(std::memory_order_acquire);
}

void SingleArmHardwareInterface::set_configured(bool state)
{
  configured_.store(state, std::memory_order_release);
}

bool SingleArmHardwareInterface::is_activated() const
{
  return activated_.load(std::memory_order_acquire);
}

void SingleArmHardwareInterface::set_activated(bool state)
{
  activated_.store(state, std::memory_order_release);
}

}  // namespace single_arm_hardware_interface

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  single_arm_hardware_interface::SingleArmHardwareInterface,
  hardware_interface::SystemInterface
)