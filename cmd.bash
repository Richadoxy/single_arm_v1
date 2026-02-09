
# joint_cmd
ros2 action send_goal /single_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    points: [
      {
        positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        time_from_start: {sec: 5, nanosec: 0}
      }
    ]
  }
}"

ros2 action send_goal /single_arm_controller/follow_joint_trajectory control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'],
    points: [
      {
        positions: [-0.1, -0.43, 1.54, -0.97, -0.00, 0.27],
        time_from_start: {sec: 5, nanosec: 0}
      }
    ]
  }
}"

ros2 control switch_controllers --activate single_arm_effort_controller --deactivate single_arm_controller
ros2 control switch_controllers --activate single_arm_controller --deactivate single_arm_effort_controller