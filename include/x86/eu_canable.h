#ifndef EU_CANABLE_H
#define EU_CANABLE_H

#ifdef __cplusplus
extern "C"
{
#endif

#ifndef EXTERNFUNC
#ifdef _WIN32
#define EXTERNFUNC __declspec(dllexport)
#else
#define EXTERNFUNC
#endif
#endif

#define CANABLE_SUCCESS (0)               /**< 执行成功 */
#define CANABLE_FAILED_DEVICEDISABLED (1) /**< 执行失败，设备不存在! */
#define CANABLE_FAILED_RECEIVEFAILED (2)  /**< 执行失败，没有接收到返回数据 */
#define CANABLE_FAILED_SENDFAILED (3)     /**< 执行失败，发送数据失败 */
#define CANABLE_FAILED_FreeFailed (4)     /**< 关闭设备失败 */

    /*!
        can数据帧结构类型
    */
    typedef struct _canable_CanMsg
    {
        unsigned id;           /**< can id */
        unsigned char data[8]; /**< can数据 */
        unsigned char dlc;     /**< 数据长度 */
        unsigned timeStamp;    /**< 时间戳 */
        bool isRTR;            /**< 是否是远程帧，true为远程帧，false为数据帧 */
        bool isExtended;       /**< 是否为扩展帧，true为扩展帧，false为标准帧 */
    } canable_CanMsg;

    /*!
        波特率
    */
    enum canable_Baudrate
    {
        canable_Baudrate_10 = 10,    /**< 波特率10 */
        canable_Baudrate_20 = 20,    /**< 波特率20 */
        canable_Baudrate_50 = 50,    /**< 波特率50 */
        canable_Baudrate_100 = 100,  /**< 波特率100 */
        canable_Baudrate_250 = 250,  /**< 波特率250 */
        canable_Baudrate_500 = 500,  /**< 波特率500 */
        canable_Baudrate_1000 = 1000 /**< 波特率1000 */
    };

    /*!
        初始化dll，在调用其他函数前必须先调用该函数进行初始化
        \param devIndex 设备索引号，第一个设备为0第二个设备为1，以此类推
        \param baudrate 波特率
        \return 成功返回CANABLE_SUCCESS,失败返回其他
    */
    EXTERNFUNC int canable_initDLL(int devIndex, canable_Baudrate baudrate);

    /*!
        关闭设备，释放资源
        \param devIndex 设备索引号，第一个设备为0第二个设备为1，以此类推
        \return 成功返回CANABLE_SUCCESS,失败返回其他
    */
    EXTERNFUNC int canable_freeDLL(int devIndex);

    /*!
        发送can数据
        \param devIndex 设备索引号，第一个设备为0第二个设备为1，以此类推
        \param msg can数据帧
        \return 成功返回CANABLE_SUCCESS,失败返回其他
    */
    EXTERNFUNC int canable_writeMessage(int devIndex, const canable_CanMsg *const msg);

    /*!
        接收can数据
        \param devIndex 设备索引号，第一个设备为0第二个设备为1，以此类推
        \param msg 用于存储接收到的can数据
        \param timeout 等待读取的时间,单位ms
        \return 成功返回CANABLE_SUCCESS,失败返回其他
    */
    EXTERNFUNC int canable_readMessage(int devIndex, canable_CanMsg *msg, int timeout = 0);

#ifdef __cplusplus
}
#endif

#endif // EU_CANABLE_H
