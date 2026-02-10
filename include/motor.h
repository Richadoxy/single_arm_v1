#ifndef MOTOR_H
#define MOTOR_H
#include "can_fd.h"
#include "can_normal.h"
#include "motor_interface.h"
#include <unistd.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#ifdef __cplusplus
extern "C"
{
#endif

typedef enum{
        open_loop = 0,
        cur_control = 1,
        vel_control = 2,
        pos_control = 3
    }CONTROL_MODE;
#define ADDR_IAP_FLAG 0x49
#define ADDR_ENABLE_FLAG 0x0A
#define ADDR_WORK_MODE 0x30
#define ADDR_CLEAR_ERROR 0x0F

class WhjMotor : public MotorInterface {
public:
    typedef enum {
        open_loop = 0,
        cur_control = 1,
        vel_control = 2,
        pos_control = 3
    } CONTROL_MODE;

    WhjMotor(uint32_t motor_id = 1, int can_dev_index = 0);

    // MotorInterface 实现
    int set_IAP() override;
    int set_absposition(double pos_rad) override;
    int set_velocity(double vel_rad_s) override;
    int set_current(double cur_A) override;
    int read_status() override;
    int enable() override;
    int disable() override;
    int clear_error() override;
    int select_mode(uint8_t mode) override;
    int decode_feedback(uint32_t received_id, const uint8_t* data, uint8_t dlc) override;
    double get_position() const override { return pos_; }
    double get_velocity() const override { return vel_; }
    double get_current() const override { return cur_; }
    int is_control_success() const override { return control_success_==1; }
    MotorType get_type() const override { return MotorType::WHJ; }
    uint32_t get_id() const override { return motor_id_; }

    // Whj 特有函数（保留供 poll 使用）
    
    int servo_control(uint32_t can_id, int32_t control_target);
    int write_register(uint8_t addr, uint8_t values, uint8_t dlc);

    // 成员变量
    uint32_t motor_id_;
    float pos_;      // rad
    float vel_;      // rpm
    float cur_;      // mA
    float last_pos_;
    int can_dev_index_;
    int control_success_;
    int error_code_;
    int enable_status_;
    int brake_status_;

private:
    int decode_motor_data(uint8_t* resp_max_data);
    int decode_motor_data2(uint8_t* resp_max_data);
};

class RmdMotor : public MotorInterface {
public:
    RmdMotor(uint32_t motor_id = 1, int can_dev_index = 0);

    // MotorInterface 实现
    int set_IAP() override { return 0; }  // RMD 无 IAP 模式
    int set_absposition(double pos_rad) override;
    int set_velocity(double vel_rad_s) override;
    int set_current(double cur_A) override;
    int read_status() override;
    int set_id(uint8_t new_id);
    int enable() override { return 0; }                    // RMD 无明确使能命令
    int disable() override;
    int clear_error() override { return 0; }               // RMD 无清错命令
    int select_mode(uint8_t mode) override; 
    
    double get_position() const override { return pos_; }
    double get_velocity() const override { return vel_; }
    double get_current() const override { return cur_; }
    int is_control_success() const override { return control_success_==1; }
    MotorType get_type() const override { return MotorType::RMD; }
    uint32_t get_id() const override { return motor_id_; }

    // Rmd 特有函数（供 poll 使用）
    int decode_feedback(uint32_t received_id, const uint8_t* data, uint8_t dlc) override;
    // 成员变量
    uint32_t motor_id_;
    float pos_;      // rad（单圈）
    float vel_;      // rpm
    float cur_;      // mA
    float last_pos_;
    int can_dev_index_;
    int control_success_;
    int error_code_;
private:
    int decode_motor_data(uint8_t* resp_max_data);
    int decode_motor_data_2(uint8_t* resp_max_data);
    int decode_motor_data_3(uint8_t* resp_max_data);
};


#ifdef __cplusplus
}
#endif
#endif