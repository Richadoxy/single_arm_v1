#ifndef TorSenor_H
#define TorSenor_H

#include <iostream>
#include <memory>
#include <ctime>
#include <cmath>
#include <time.h>
#include <thread>
#include <signal.h>
#include <fstream>
#include "can_normal.h"
#include <unistd.h>
#include <sys/time.h>
#include <sys/select.h>
#include <map>
#include <mutex>

class TorqueSensor {
public:


    TorqueSensor(uint32_t sensor_id=7,  int can_dev_index=0);
    ~TorqueSensor();
    int read_sensor();
    unsigned int sensor_id_;
    
    void decode_sensor_data(const unsigned char buffer[]);
    double get_torque() const { return torque_data_; }
private:
    int control_success_ = 0;
    int can_channel_;
    double torque_data_;
};

#endif // EyoTorSenor_H