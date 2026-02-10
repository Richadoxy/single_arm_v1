cmake_minimum_required(VERSION 3.10.0)

project(single_arm_v1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64" OR CMAKE_SYSTEM_PROCESSOR MATCHES "arm64")
    message(STATUS "Building for Linux-ARM architecture")
    add_definitions(-DARM_ARCH)
    set(ARCH_ARM64 TRUE)
else() 
    message(STATUS "Building for Linux-x86 architecture")
    set(ARCH_ARM64 FALSE)
endif()

if(NOT DEFINED CANFD_LIB_PATH)
    if(ARCH_ARM64)
        message(STATUS "ARM64 architecture detected, using arm64 library path")
        set(CANFD_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/canfd/arm64)
    else()
        message(STATUS "x86 architecture detected, using x86 CAN FD library path")
        set(CANFD_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/canfd/x86)
    endif()
endif()
if(NOT DEFINED CANNORMAL_LIB_PATH)
    if(ARCH_ARM64)
        set(CANNORMAL_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/can/arm64)    
    else()
        message(STATUS "x86 architecture detected, using x86 CAN NORMAL library path")
        set(CANNORMAL_LIB_PATH ${PROJECT_SOURCE_DIR}/lib/can/x86)    
    endif()   
endif()

if(ARCH_ARM64)
    # ARM64架构：优先包含arm64目录
    include_directories(
        ${PROJECT_SOURCE_DIR}/include/arm64  # 架构专用头文件
        ${PROJECT_SOURCE_DIR}/include        # 通用头文件
    )
else()
    # 其他架构：标准包含路径
    include_directories(
        ${PROJECT_SOURCE_DIR}/include/x86  # 架构专用头文件
        ${PROJECT_SOURCE_DIR}/include 
    )
endif()
find_package(yaml-cpp)
function(add_custom_demo demo_name) 
    add_executable(${demo_name}
                    ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_fd.c
                    ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_normal.cpp
                    ${PROJECT_SOURCE_DIR}/src/hardware/motor.cpp
                    ${PROJECT_SOURCE_DIR}/src/hardware/torque_sensor.cpp
                    ${PROJECT_SOURCE_DIR}/src/${demo_name}.cpp
                )
    target_link_libraries(${demo_name} ${CANFD_LIB_PATH}/libusbcanfd.so -lm -lpthread -lrt -Wall)
    # target_link_libraries(${demo_name} ${CANNORMAL_LIB_PATH}/libcontrolcan.so pthread)
    target_link_libraries(${demo_name} ${CANNORMAL_LIB_PATH}/libeu_canable.so pthread)
    target_link_libraries(${demo_name} yaml-cpp)
    target_compile_definitions(${demo_name} PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endfunction()

function(add_custom_test test_name) 
    add_executable(${test_name}
                    ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_fd.c
                    ${PROJECT_SOURCE_DIR}/src/usb_to_can/can_normal.cpp
                    ${PROJECT_SOURCE_DIR}/src/plan/polynomial_trajectory.cpp
                    ${PROJECT_SOURCE_DIR}/src/hardware/motor.cpp
                    ${PROJECT_SOURCE_DIR}/src/hardware/torque_sensor.cpp
                    ${PROJECT_SOURCE_DIR}/src/plan/ccg_controller.cpp
                    ${PROJECT_SOURCE_DIR}/src/plan/ccg_fric_controller.cpp
                    ${PROJECT_SOURCE_DIR}/src/hardware/robot_arm.cpp
                    ${PROJECT_SOURCE_DIR}/src/${test_name}.cpp

                )
    target_link_libraries(${test_name} ${CANFD_LIB_PATH}/libusbcanfd.so -lm -lpthread -lrt -Wall)
    target_link_libraries(${test_name} ${CANNORMAL_LIB_PATH}/libeu_canable.so pthread)
    target_link_libraries(${test_name} yaml-cpp)
    target_compile_definitions(${test_name} PRIVATE PROJECT_SOURCE_DIR="${PROJECT_SOURCE_DIR}")
endfunction()

add_custom_test(cur_control)
add_custom_test(go_home)
add_custom_test(enable)
add_custom_test(pos_control)
add_custom_test(test_torque_sensor)
add_custom_test(grav_comp)
add_custom_test(grav_comp_with_fric)
add_custom_test(grav_comp_with_tqs)