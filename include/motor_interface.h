#ifndef MOTOR_INTERFACE_H
#define MOTOR_INTERFACE_H

#include <cstdint>
#include <cstddef>

// 电机类型枚举
enum class MotorType {
    WHJ = 0,
    RMD = 1,
    
};

// 抽象电机接口
class MotorInterface {
public:
    virtual ~MotorInterface() = default;  // 虚析构函数，确保派生类正确析构

    // ==================== 控制命令 ====================
    virtual int set_IAP() = 0;  // 可选：设置 IAP 模式（仅 Whj 需要，Rmd 可空实现返回 0）
    // 设置绝对位置（rad）
    virtual int set_absposition(double pos_rad) = 0;

    // 设置速度（rpm）
    virtual int set_velocity(double vel_rad_s) = 0;

    // 设置电流（A）
    virtual int set_current(double cur_A) = 0;

    // 触发读取状态（发送读命令或控制命令以触发反馈）
    virtual int read_status() = 0;

    // 使能电机（Whj 有明确使能命令，Rmd 可空实现）
    virtual int enable() = 0;

    // 禁用电机
    virtual int disable() = 0;

    // 清错（Whj 有，Rmd 可空实现）
    virtual int clear_error() = 0;

    // 模式选择（仅 Whj 需要，Rmd 可空实现返回 0）
    virtual int select_mode(uint8_t mode) = 0;

    virtual int decode_feedback(uint32_t received_id, const uint8_t* data, uint8_t dlc) = 0;
    // ==================== 状态读取 ====================
    virtual double get_position() const = 0;   // rad
    virtual double get_velocity() const = 0;   // rpm
    virtual double get_current() const = 0;    // mA
    virtual int is_control_success() const = 0;
    // ==================== 辅助信息 ====================
    virtual MotorType get_type() const = 0;    // 返回电机类型
    virtual uint32_t get_id() const = 0;       // 返回电机 CAN ID（用于 map 轮询）
    
    // 可选：如果需要更多状态（如错误码、使能状态等）
    // virtual int get_error_code() const = 0;
    // virtual int get_enable_status() const = 0;
};

#endif // MOTOR_INTERFACE_H