

#ifndef CCG_CONTROLLER_H
#define CCG_CONTROLLER_H

#include <vector>
#include <cmath>  // sin, cos, pow 等

/**
 * @brief 单臂机器人的 CCG 补偿控制器类
 * 
 * 输入：关节位置 q (6维), 关节速度 q_dot (6维)
 * 输出：补偿扭矩 tau (6维) = Coriolis + Centrifugal + Gravity 项
 */
class CCGController {
private:
    std::vector<double> L;     // 连杆长度参数（固定值）
    std::vector<double> beta;  // 最小惯性参数集 beta（固定值，已辨识）
    
public:
    /**
     * @brief 构造函数
     * 初始化 L 和 beta 的固定值
     */
    CCGController();

    /**
     * @brief 计算补偿扭矩 tau = C(q, \dot{q})\dot{q} + G(q)
     * @param q      当前关节位置向量 (size=6)
     * @param q_dot  当前关节速度向量 (size=6)
     * @param tau    输出：计算得到的补偿扭矩向量 (size=6)
     */
    void computeTau(const std::vector<double>& q,
                    const std::vector<double>& q_dot,
                    std::vector<double>& tau);
};

#endif // CCG_CONTROLLER_H