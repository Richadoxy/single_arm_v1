# Single Arm V1 Control Based on ROS2 Control and MoveIt

## Overview
This project provides control for a single-arm robot (V1) using ROS2 Control and MoveIt. It includes the following packages:
- `single_arm_description`: Robot description files (URDF, SRDF, etc.).
- `single_arm_hardware_interface`: Hardware interface for motor drivers and CAN communication.
- `single_arm_control`: Control configurations and launch files.

## Tested Environment
- Ubuntu 22.04
- ROS2 Humble

## Installation
1. Make the CAN initialization script executable:
   ```
   chmod +x ./can_init.bash
   ```

2. Build the workspace:
   ```
   colcon build
   ```

3. Source the setup script:
   ```
   source install/setup.bash
   ```

## Usage
To run the system:

1. Initialize the CAN interface:
   ```
   ./can_init.bash
   ```

2. Launch the real hardware configuration (without gripper, in real mode):
   ```
   ros2 launch single_arm_description real_hw.launch.py use_gripper:=false sim:=false
   ```

## Notes
- Ensure your CAN interfaces (e.g., can0, can1) are properly configured before running.
- For simulation mode, set `sim:=true`.
- If using a gripper, set `use_gripper:=true`.