#include "torque_sensor.h"

TorqueSensor::TorqueSensor(uint32_t sensor_id,  int can_dev_index) {
    sensor_id_ = sensor_id;
    can_channel_ = can_dev_index;
}

TorqueSensor::~TorqueSensor() {
    // 清理资源
}


int TorqueSensor::read_sensor() {
    unsigned char send_data[8];
    unsigned char receive_data[8];
    unsigned int receive_id = 0;
    for (int i = 0; i < 8; ++i)
        send_data[i] = 0x00;
    send_data[0] = 0x55;
    control_success_ = 0;
    uint32_t sensor_id = sensor_id_ + 0x100;
    return can_normal_send(can_channel_,sensor_id, send_data);
}

void TorqueSensor::decode_sensor_data(const unsigned char buffer[]) {
    control_success_ = 1;
    int16_t data = ((int16_t)buffer[0] << 8) | ((int16_t)buffer[1]);
    torque_data_ = data * 0.001f;
}
