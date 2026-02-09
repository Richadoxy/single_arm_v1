#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <thread>
#include <chrono>

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("random_move_node");

  // 创建 MoveGroupInterface（规划组名必须和你的 SRDF 一致）
  moveit::planning_interface::MoveGroupInterface move_group(node, "single_arm_group");

  // 设置规划参数（提升成功率和安全性）
  move_group.setPlannerId("RRTConnectkConfigDefault");
  move_group.setPlanningTime(5.0);
  move_group.setNumPlanningAttempts(5);
  move_group.setMaxVelocityScalingFactor(0.5);     // 速度放慢 50%，真实臂更安全
  move_group.setMaxAccelerationScalingFactor(0.5);

  RCLCPP_INFO(node->get_logger(), "开始移动到随机有效关节位置...");

  bool success = false;
  int attempts = 0;
  const int max_attempts = 10;

  while (!success && attempts < max_attempts && rclcpp::ok())
  {
    attempts++;
    RCLCPP_INFO(node->get_logger(), "第 %d 次尝试生成随机目标...", attempts);

    // 生成随机关节目标（MoveIt 自动在限位内 + 碰撞检查）
    move_group.setRandomTarget();

    // 规划
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool plan_success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (plan_success)
    {
      RCLCPP_INFO(node->get_logger(), "规划成功！准备执行...");
      // 执行
      moveit::core::MoveItErrorCode execute_result = move_group.execute(plan);
      if (execute_result == moveit::core::MoveItErrorCode::SUCCESS)
      {
        RCLCPP_INFO(node->get_logger(), "机械臂成功移动到随机有效位置！");
        success = true;
      }
      else
      {
        RCLCPP_WARN(node->get_logger(), "执行失败，重试...");
      }
    }
    else
    {
      RCLCPP_WARN(node->get_logger(), "规划失败（可能自碰撞），重新生成随机目标...");
    }
  }

  if (!success)
  {
    RCLCPP_ERROR(node->get_logger(), "达到最大重试次数，未能找到有效随机目标。");
  }

  rclcpp::shutdown();
  return 0;
}