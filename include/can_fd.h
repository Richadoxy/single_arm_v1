#ifndef CAN_FD_H
#define CAN_FD_H
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <stdint.h>



#ifdef __cplusplus
extern "C"
{
#endif
#define USBCANFD  33
#define CANFD_MAX_DATA_LEN 64
#define DEFAULT_TIMEOUT_MS 1000
#ifdef TEST_MODE
#define DEBUG_PRINT(...) printf(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#endif
typedef struct {
    uint32_t arb_brate;  // 仲裁段波特率（需预定义参数）
    uint32_t data_brate; // 数据段波特率（需预定义参数）
    int termination;    // 是否启用终端电阻
} CANFD_Config;

int canfd_init(int dev_idx, int chn_idx);
int canfd_send(int dev_idx, int chn_idx, uint32_t id, const uint8_t* data, uint8_t len);
int canfd_receive(int dev_idx, int chn_idx, uint32_t* id, uint8_t* data, uint8_t* len, int timeout_ms);
void canfd_close(int dev_idx, int chn_idx);
#ifdef __cplusplus
}
#endif

#endif