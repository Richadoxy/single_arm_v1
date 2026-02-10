#include "can_normal.h"
int can_normal_init(unsigned int device_id)
{
    if (CANABLE_SUCCESS != canable_initDLL(device_id, canable_Baudrate_1000))
    {
        printf("Init CANABLE Device %d Failed\n", device_id);
        return -1;
    }
    else
    {
        printf("CANABLE Device %d init success\n", device_id);
    }
    return 0;
}

int can_normal_send(unsigned int device_id,unsigned int can_id, const unsigned char* data)
{
    canable_CanMsg write_msg;
    write_msg.isExtended = 0;
    write_msg.isRTR = 0;
    write_msg.dlc = 8;
    write_msg.id = can_id;
    memcpy(write_msg.data, data, 8);
    // for (int i = 0; i < 8; i++) {
    //     printf("can id: 0x%03X ", can_id);
    //     printf("CAN Send Data[%d]: 0x%02X\n", i, write_msg.data[i]);
    // }
    int ret = canable_writeMessage(device_id, &write_msg);
    if (ret != CANABLE_SUCCESS) {
        printf("Send CANABLE Message Failed,Error Code: %d\n", ret);
        return -1;
    }
    return 0;
}
int can_normal_receive(unsigned int device_id, unsigned char* data, unsigned int *receive_id)
{
    canable_CanMsg read_msg;
    int ret = canable_readMessage(device_id, &read_msg, 10);
    if (ret != CANABLE_SUCCESS) {
        //printf("Device_id %d Read CANABLE Message Failed,Error Code: %d\n",device_id, ret);
        return -1;
    }
    if (read_msg.dlc == 8)
        memcpy(data, read_msg.data, read_msg.dlc);
    *receive_id = read_msg.id;
    return 0;
}

int can_normal_close(unsigned int device_id)
{
    if (CANABLE_SUCCESS != canable_freeDLL(device_id))
    {
        printf("Close CANABLE Device %d Failed\n", device_id);
        return -1;
    }
    else
    {
        printf("CANABLE Device %d close success\n", device_id);
    }
    return 0;
}

