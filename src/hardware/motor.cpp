#include "motor.h"  

#define TIMEOUT_MS 10

// Constructor
WhjMotor::WhjMotor(uint32_t motor_id, int can_dev_index)
    : motor_id_(motor_id), pos_(0.0f), vel_(0.0f), cur_(0.0f), last_pos_(0.0),
      can_dev_index_(can_dev_index), control_success_(1) {}

// Member function implementations
int WhjMotor::set_IAP() {
    return write_register(ADDR_IAP_FLAG, 0x00, 3);
}

int WhjMotor::enable() {

    return write_register(ADDR_ENABLE_FLAG, 0x01, 3);
}

int WhjMotor::disable() {
    return write_register(ADDR_ENABLE_FLAG, 0x00, 3);
}

int WhjMotor::clear_error() {
    return write_register(ADDR_CLEAR_ERROR, 0x01, 3);
}

int WhjMotor::select_mode(uint8_t control_mode) {
    return write_register(ADDR_WORK_MODE, control_mode, 3);
}

int WhjMotor::set_velocity(double vel_rad_s) {
    uint32_t can_id = motor_id_ + 0x300;
    double vel_rpm = vel_rad_s /3.1415926 * 30.0;
    int32_t vel_units = (int32_t)(vel_rpm * 500);
    return servo_control(can_id, vel_units);
}

int WhjMotor::set_current(double cur_A) {
    uint32_t can_id = motor_id_ + 0x400;
    int32_t cur_units = (int32_t)(cur_A*1000.0);
    // std::cout << " motor_id_ "<< motor_id_ << "can_units" << cur_units <<std::endl;
    return servo_control(can_id, cur_units);
}

int WhjMotor::set_absposition(double pos_rad) {
    double pos_deg = pos_rad * 180.0 / 3.141592653589793;
    uint32_t can_id = motor_id_ + 0x200;
    int32_t pos_units = (int32_t)(pos_deg * 10000);
    return servo_control(can_id, pos_units);
}

int WhjMotor::read_status(void) {
    uint32_t can_id = motor_id_ + 0x600;
    uint8_t dlc = 0;
    uint8_t send_data;
    uint32_t resp_Id = 0;
    uint8_t resp_max_data[64];
    uint8_t resp_dlc = 0;
    uint8_t send_status = canfd_send(can_dev_index_, 0, can_id, &send_data, dlc);
    control_success_ = 0;
    if (send_status < 0) {
        return send_status;
    }
    return 0;
}

int WhjMotor::write_register(uint8_t addr, uint8_t values, uint8_t dlc) {
    uint8_t write_cmd = 0x02;
    uint8_t send_data[3] = {write_cmd, addr, values};
    uint32_t resp_Id = 0;
    uint8_t resp_max_data[64];
    uint8_t resp_dlc = 0;
    control_success_ = 0; 
    uint8_t send_status = canfd_send(can_dev_index_, 0, motor_id_, send_data, dlc);
    if (send_status < 0) {
        return send_status;
    }
    return 0;
}

int WhjMotor::decode_motor_data(uint8_t* resp_max_data) {
    pos_ = static_cast<int32_t>((resp_max_data[11] << 24) | (resp_max_data[10] << 16) |
                               (resp_max_data[9] << 8) | resp_max_data[8]) * 0.0001 * 3.1415926 / 180.0;
    vel_ = static_cast<int32_t>((resp_max_data[7] << 24) | (resp_max_data[6] << 16) |
                               (resp_max_data[5] << 8) | resp_max_data[4]) * 0.02f / 30.0 *3.1415926;
    cur_ = static_cast<int32_t>((resp_max_data[3] << 24) | (resp_max_data[2] << 16) |
                               (resp_max_data[1] << 8) | resp_max_data[0]) * 0.001f;
    return 0;
}

int WhjMotor::decode_motor_data2(uint8_t* resp_max_data) {
    // 根据您原代码保留
    pos_ = static_cast<int32_t>((resp_max_data[11] << 24) | (resp_max_data[10] << 16) |
                               (resp_max_data[9] << 8) | resp_max_data[8]) * 0.0001 * 3.1415926 / 180.0;
    error_code_ = (resp_max_data[1] << 8) | resp_max_data[0];
    enable_status_ = resp_max_data[6];
    brake_status_ = resp_max_data[7];
    cur_ = static_cast<int32_t>((resp_max_data[15] << 24) | (resp_max_data[14] << 16) |
                               (resp_max_data[13] << 8) | resp_max_data[12]) * 0.001f;
    return 0;
}

int WhjMotor::decode_feedback(uint32_t received_id, const uint8_t* data, uint8_t dlc) {
    if (received_id == motor_id_ + 0x500) {
        decode_motor_data(const_cast<uint8_t*>(data));
        control_success_ = 1;
    } else if (received_id == motor_id_ + 0x700) {
        decode_motor_data2(const_cast<uint8_t*>(data));
        control_success_ = 1;
    } else if (received_id == motor_id_ + 0x100) {
        control_success_ = 1;
    }
    return 0;
}

int WhjMotor::servo_control(uint32_t can_id, int32_t controlTarget) {

    uint8_t send_data[4];
    send_data[0] = (uint8_t)(controlTarget & 0xFFFF);
    send_data[1] = (uint8_t)((controlTarget >> 8) & 0xFFFF);
    send_data[2] = (uint8_t)((controlTarget >> 16) & 0xFFFF);
    send_data[3] = (uint8_t)((controlTarget >> 24) & 0xFFFF);
    uint8_t dlc = 4;
    uint32_t resp_Id = 0;
    uint8_t resp_max_data[64];
    uint8_t resp_dlc = 0;
    control_success_ = 0;
    uint8_t send_status = canfd_send(can_dev_index_, 0, can_id, send_data, dlc);
    if (send_status < 0) {
        return send_status;
    }
    return 0;
}

RmdMotor::RmdMotor(uint32_t motor_id, int can_dev_index)
    : motor_id_(motor_id), pos_(0.0f), vel_(0.0f), cur_(0.0f), last_pos_(0.0f),
      can_dev_index_(can_dev_index),control_success_(1) {}

// Disable motor (send 0x80 command, common in RMD series)
int RmdMotor::disable() {
    uint8_t send_data[8] = {0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    uint32_t can_id = motor_id_;
    uint8_t send_status = canfd_send(can_dev_index_, 0, can_id, send_data, 8);
    if (send_status < 0) {
        return send_status;
    }
    return 0;
}
int RmdMotor::set_id(uint8_t new_id) {
    uint8_t send_data[8] = {0x20, 0x05, 0x00, 0x00, new_id, 0x00, 0x00, 0x00};
    uint32_t can_id = motor_id_;
    uint8_t send_status = can_normal_send(can_dev_index_, 0x300, send_data);
    if (send_status < 0) {
        std::cout << "Set ID failed: Error sending set ID command." << std::endl;
        return send_status;
    }
    uint8_t receive_data[8];
    uint8_t receive_dlc = 0;
    uint32_t receive_id = 0;
    uint8_t receive_status = can_normal_receive(can_dev_index_,receive_data, &receive_id);
    if (receive_status != 0 || receive_id != (0x300) || receive_data[0] != 0x79) {
        std::cout << "RECEIVED ID: " << std::hex << receive_id << std::dec << std::endl;
        std::cout << "Set ID failed: No acknowledgment received." << std::endl;
        for (int i = 0; i < 8; i++) {
            std::cout << "Received Data[" << i << "]: " << std::hex << static_cast<int>(receive_data[i]) << std::dec << std::endl;
        }
        return -1; // Error in receiving acknowledgment
    }
    
    return 0;
}

// Set velocity (0xA2 command)
int RmdMotor::set_velocity(double vel_rad_s) {

    double vel_dps = vel_rad_s *180/3.1415926;
    int32_t speed_units = (int32_t)(vel_dps * 100.0);  // rpm -> dps -> 0.01 dps/LSB
    uint8_t send_data[8];
    send_data[0] = 0xA2;
    send_data[1] = 0x00;  // maxTorque = 0 (no torque limit)
    send_data[2] = 0x00;
    send_data[3] = 0x00;
    send_data[4] = (uint8_t)(speed_units & 0xFF);
    send_data[5] = (uint8_t)((speed_units >> 8) & 0xFF);
    send_data[6] = (uint8_t)((speed_units >> 16) & 0xFF);
    send_data[7] = (uint8_t)((speed_units >> 24) & 0xFF);

    uint32_t can_id = motor_id_+0x140;
    control_success_ = 0;
    uint8_t send_status = can_normal_send(can_dev_index_,  can_id, send_data);
    if (send_status < 0) {
        return send_status;
    }
    return 0;
}

int RmdMotor::select_mode(uint8_t control_mode) 
{
    if (control_mode == cur_control) {
        set_current(0.0);  
        return 0;
    } else if (control_mode == vel_control) {
        // No explicit mode selection command for velocity control in RMD
        set_velocity(0.0);
        return 0;
    } else if (control_mode == pos_control) {
        // No explicit mode selection command for position control in RMD
        set_velocity(0.0);
        return 0;
    } else {
        std::cout << "RmdMotor " << motor_id_ << " select_mode: Unsupported control mode " << (int)control_mode << std::endl;
        return -1;  // Unsupported mode
    }
    
    return 0;
}
        

// Set current/torque (0xA1 command, assumed format similar to common RMD)
int RmdMotor::set_current(double cur_A) {
    int16_t cur_units = (int16_t)(cur_A / 0.01);  
    uint8_t send_data[8] = {0xA1, 0x00, 0x00, 0x00,
                            (uint8_t)(cur_units & 0xFF),
                            (uint8_t)((cur_units >> 8) & 0xFF),
                            0x00, 0x00};

    uint32_t can_id = motor_id_+0x140;
    control_success_ = 0;
    uint8_t send_status = can_normal_send(can_dev_index_,  can_id, send_data);
    if (send_status < 0) {
        return send_status;
    }
    return 0;
}

// Set absolute position (0xA4 multi-turn position command)
int RmdMotor::set_absposition(double pos_rad) {
    double pos_deg = pos_rad * 180.0 / 3.141592653589793;
    int32_t pos_units = (int32_t)(pos_deg * 100.0);  // rad -> deg -> 0.01 deg/LSB
    int16_t max_speed = 20000;  // 0 means no speed limit (or use acceleration planning)

    uint8_t send_data[8];
    send_data[0] = 0xA4;
    send_data[1] = 0x00;
    send_data[2] = (uint8_t)(max_speed & 0xFF);
    send_data[3] = (uint8_t)(max_speed >> 8);
    send_data[4] = (uint8_t)(pos_units & 0xFFFF);
    send_data[5] = (uint8_t)((pos_units >> 8) & 0xFFFF);
    send_data[6] = (uint8_t)((pos_units >> 16) & 0xFFFF);
    send_data[7] = (uint8_t)((pos_units >> 24) & 0xFFFF);
    
    uint32_t can_id = motor_id_ + 0x140;
    control_success_ = 0;
    uint8_t send_status = can_normal_send(can_dev_index_,  can_id, send_data);
    if (send_status < 0) {
        return send_status;
    }
    return 0;
}

// Read status (send zero velocity command to trigger feedback)
int RmdMotor::read_status(void) {
    uint8_t send_data[8];
    uint8_t READ_DATA_CMD = 0x92;
    send_data[0] = READ_DATA_CMD;
    send_data[1] = 0x00;
    send_data[2] = 0x00;
    send_data[3] = 0x00;
    send_data[4] = 0x00;
    send_data[5] = 0x00;
    send_data[6] = 0x00;
    send_data[7] = 0x00;

    uint32_t can_id = motor_id_ + 0x140;
    control_success_ = 0;
    uint8_t send_status = can_normal_send(can_dev_index_, can_id, send_data);
    //std::cout << "RmdMotor " << motor_id_ << " read_status send cmd" << std::endl;
    if (send_status < 0) {
        std::cout << "RmdMotor " << motor_id_ << " read_status send error" << std::endl;
        return send_status;
    }

    return 0;
}

// Decode feedback data (common format for A1/A2/A4 replies)
int RmdMotor::decode_motor_data(uint8_t* resp_max_data) {
    // DATA[1]: temperature (not used here)
    int16_t iq = (int16_t)((resp_max_data[3] << 8) | resp_max_data[2]);
    int16_t speed = (int16_t)((resp_max_data[5] << 8) | resp_max_data[4]);
    int16_t angle_deg = (int16_t)((resp_max_data[7] << 8) | resp_max_data[6]);
    
    cur_ = iq * 0.01f;                // A
    vel_ = speed *3.1415926/180;        
    pos_ = (float)angle_deg * (3.141592653589793f / 180.0f);  // deg -> rad 
    // std::cout << "RmdMotor " << motor_id_ << " decode_motor_data pos: " << pos_
    //           << " rad, vel: " << vel_ << " rpm, cur: " << cur_ << " A" << std::endl;
    return 0;
}

int RmdMotor::decode_motor_data_2(uint8_t* resp_max_data) {
    // DATA[1]: temperature (not used here)

    int32_t angle_deg = (int16_t)((resp_max_data[7] << 24) | resp_max_data[6] << 16 |
                                  resp_max_data[5] << 8 | resp_max_data[4]);
    
    pos_ = angle_deg * 0.01f * (3.141592653589793f / 180.0f);  // deg -> rad
    return 0;
}
int RmdMotor::decode_motor_data_3(uint8_t* resp_max_data) {
    // DATA[1]: temperature (not used here)

    int16_t error_  = (int16_t)((resp_max_data[7] << 8) | resp_max_data[6]);
    printf("RmdMotor %d decode_motor_data_3 error code: %d\n", motor_id_, error_);
    error_code_ = error_;
    return 0;
}
int RmdMotor::decode_feedback(uint32_t received_id, const uint8_t* data, uint8_t dlc) {
    if (received_id == motor_id_ + 0x240) {
        if (data[0]!= 0x92 && data[0]!=0x9A)
        {
            decode_motor_data(const_cast<uint8_t*>(data)); 
        } else if (data[0] == 0x92)
        {
            decode_motor_data_2(const_cast<uint8_t*>(data));
        }else
        {
            decode_motor_data_3(const_cast<uint8_t*>(data));
        }
        
        control_success_ = 1;
    }
    return 0;
}