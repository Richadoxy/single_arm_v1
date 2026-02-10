# Single Arm Gravity & Friction Compensation Control

A hardware interface and joint space control for a single arm v1 robotic arm using CAN/CAN-FD communication. WHJ Motor for 1-3 joint(canfd), RMD Motor for 4-6 joint(can).

## Project Overview

This project provides a single-arm control system with the following core features:

- Gravity + friction compensation
- Joint-space trajectory generation and execution (5th-order polynomial)
- Support for multiple motor types (WHJ via CAN-FD, RMD via CAN)
- Optional torque sensor integration (CAN-based)
- YAML-based configuration


## Hardware Requirements
- Motor:
  - WHJ-30 series (CAN-FD supported)
  - RMD-x4-36 series (CAN supported)
- CAN/CAN-FD interface card (ZLG-canfd-mini, eu-canable)
- Power supply matching motor rating
- (Optional) Torque sensors on some joints (CAN communication)

## Software Dependencies

- C++17
- yaml-cpp
- CMake
- Linux (Ubuntu 20.04 / 22.04 recommended)

```bash
# Install yaml-cpp (Ubuntu example)
sudo apt update
sudo apt install libyaml-cpp-dev
```
## Build & Run

```bash
## BUILD PROJECT
mkdir build && cd build
cmake ..
make -j4

## Common executables
# 1. Enable motors and print status (for debugging)
sudo ./enable

# 2. Move to home position (make sure home is safe!)
sudo ./go_home

# 3. Start gravity + friction compensation control (main program)
sudo ./grav_comp_with_fric
```

