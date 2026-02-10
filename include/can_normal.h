#ifndef CAN_NORMAL_H
#define CAN_NORMAL_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include "eu_canable.h"
#ifdef __cplusplus
extern "C"
{
#endif

int can_normal_init(unsigned int device_id);
int can_normal_send(unsigned int device_id,unsigned int can_id, const unsigned char* data);
int can_normal_receive(unsigned int device_id, unsigned char* data, unsigned int *receive_id);
int can_normal_close(unsigned int device_id);
#ifdef __cplusplus
}
#endif
#endif