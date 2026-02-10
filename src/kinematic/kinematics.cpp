#include "kinematics.h"
#include <cmath>
#include <iostream>
#include <cstdio>

constexpr double SINGULARITY_THRESHOLD = 100000.0;
constexpr double GRAVITY = 9.81;

// 正向运动学：关节角到笛卡尔位姿（世界系）
int kinematics::forwardKinematics(const Robot& robot, const double q_in[NUM_JOINTS_PER_ARM], double cart_pos[6]) {
    double q[NUM_JOINTS_PER_ARM];

    // 检查限位并加偏移
    for (int i = 0; i < NUM_JOINTS_PER_ARM; i++) {
        if (q_in[i] > robot.joint_limit.joint_positive_limit[i] ||
            q_in[i] < robot.joint_limit.joint_negative_limit[i]) {
            std::cout << "关节 " << i << " 超限！(FK计算)" << std::endl;
            return 1;
        }
        q[i] = q_in[i] + robot.mdh.offset[i];
    }

    // 计算变换矩阵
    matrix::Matrix<double, 4, 4> T;
    T.identity();
    matrix::SquareMatrix<double, 4> Ti;
    for (int i = 0; i < NUM_JOINTS_PER_ARM; i++) {
        robotics_math::getTransformationMatrixMDH(q[i], robot.mdh.alpha[i],
                                                 robot.mdh.a[i], robot.mdh.d[i], Ti);
        T = T * Ti;
    }

    // 基系位姿
    double base_pos[6] = {0.0};
    robotics_math::Mat2Eula(T, base_pos);

    // 转世界系
    baseToWorld(robot, cart_pos, base_pos);
    return 0;
}

// 奇异鲁棒逆运动学：笛卡尔位姿到关节角（世界系）
int kinematics::inverseKinematicsSingularityRobust(const Robot& robot, const double dest_cart[6],
                                                  double tol, int max_iter,
                                                  const double last_q[NUM_JOINTS_PER_ARM], double q_out[NUM_JOINTS_PER_ARM]) {
    int itr = 0;
    matrix::Matrix<double, 6, 1> dxe;
    matrix::Matrix<double, 6, NUM_JOINTS_PER_ARM> J;
    double lambda = 1.0;
    double sum = 0.0;
    double sum_dq[NUM_JOINTS_PER_ARM] = {0.0};
    matrix::Matrix<double, NUM_JOINTS_PER_ARM, 6> pinvJ;
    double q_cur[NUM_JOINTS_PER_ARM];

    // 初始化关节角
    for (int i = 0; i < NUM_JOINTS_PER_ARM; i++) {
        q_cur[i] = last_q[i] + robot.mdh.offset[i];
        q_out[i] = q_cur[i];
        //std::cout << "初始关节角 " << i << ": " << q_out[i] << std::endl;
    }

    // 目标转基系
    double dest_base[6] = {0.0};
    worldToBase(robot, dest_base, dest_cart);

    // 计算初始误差和雅可比
    calculateErrorJacobian(robot, q_out, dest_base, dxe, J);
    for (int i = 0; i < 6; i++) sum += dxe(i, 0) * dxe(i, 0);
    double norm = std::sqrt(sum);


    // 可操作度
    matrix::SquareMatrix<double, 6> ind = J * J.T();
    double meas = std::sqrt(std::abs(matrix::det(ind)));  // 确保det非负
    //std::cout << "初始可操作度: " << meas << std::endl;

    // 若奇异区，使用鲁棒解
    if (meas < SINGULARITY_THRESHOLD) {
        double sr_k = 1.5 * (1.0 - meas / SINGULARITY_THRESHOLD) * (1.0 - meas / SINGULARITY_THRESHOLD);
        auto I6 = matrix::eye<double, 6>();
        matrix::SquareMatrix<double, 6> temp = ind + sr_k * I6;
        matrix::Matrix<double, NUM_JOINTS_PER_ARM, 6> J_inv = J.T() * matrix::inv(temp);
        matrix::Matrix<double, NUM_JOINTS_PER_ARM, 1> dq = J_inv * dxe;
        for (int i = 0; i < NUM_JOINTS_PER_ARM; i++) {
            q_out[i] += lambda * dq(i, 0);
            sum_dq[i] += lambda * dq(i,  0);
        }
    } else {
        // 迭代求解
        while (norm > tol && itr < max_iter) {
            pinvJ.setZero();
            pinvJ = J.T() * ind.I();
            matrix::Matrix<double, NUM_JOINTS_PER_ARM, 1> dq = pinvJ * dxe;
            for (int i = 0; i < NUM_JOINTS_PER_ARM; i++) {
                q_out[i] += lambda * dq(i, 0);
                sum_dq[i] += lambda * dq(i, 0);
            }
            calculateErrorJacobian(robot, q_out, dest_base, dxe, J);
            

            ind = J * J.T();
            meas = std::sqrt(std::abs(matrix::det(ind)));
            //std::cout << "当前可操作度: " << meas << std::endl;

            if (meas < SINGULARITY_THRESHOLD) {
                //std::cout << "进入奇异区，退出迭代" << std::endl;
                break;
            }

            sum = 0.0;
            for (int i = 0; i < 6; i++) sum += dxe(i, 0) * dxe(i, 0);
            norm = std::sqrt(sum);
            //std::cout << "当前误差范数: " << norm << std::endl;
            itr++;
        }

        //std::cout << "最终迭代次数: " << itr << std::endl;
        if (norm >= tol && itr >= max_iter) {
            std::cout << "!!!!!!!!!!!!超过迭代次数，无解!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            return 4; // 超迭代
        }
    }

    // 后处理：与旧代码保持一致
    for (int i = 0; i < NUM_JOINTS_PER_ARM; i++) {
        q_out[i] -= robot.mdh.offset[i];  // 移除偏移
        if (q_out[i] > PI) q_out[i] -= 2 * PI;
        else if (q_out[i] < -PI) q_out[i] += 2 * PI;
        if (std::fabs(q_out[i] - last_q[i]) < 0.000001) q_out[i] = last_q[i];  // 避免微小误差
    }

    //printf("SRq_out: %f,%f,%f,%f,%f,%f,%f\n", q_out[0], q_out[1], q_out[2], q_out[3], q_out[4], q_out[5], q_out[6]);

    // 验证正向运动学
    double next_pos[6];
    forwardKinematics(robot, q_out, next_pos);
    if (std::fabs(next_pos[0] - dest_cart[0]) > 50 ||
        std::fabs(next_pos[1] - dest_cart[1]) > 50 ||
        std::fabs(next_pos[2] - dest_cart[2]) > 50) {
        printf("!!!!!!!!!!!!位置误差过大，无解!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n");
        return 5; // 位置误差过大
    }

    return 0;
}

void kinematics::calculateErrorJacobian(const Robot& robot, const double *q, const double *desEular, 
                                       matrix::Matrix<double, 6, 1> &dxe, 
                                       matrix::Matrix<double, 6, NUM_JOINTS_PER_ARM> &J) {
    // 计算当前位姿
    matrix::Matrix<double, 4, 4> T;
    T.identity();
    matrix::SquareMatrix<double, 4> Ti;
    for (int i = 0; i < NUM_JOINTS_PER_ARM; ++i) {
        robotics_math::getTransformationMatrixMDH(q[i], robot.mdh.alpha[i], robot.mdh.a[i], robot.mdh.d[i], Ti);
        T = T * Ti;
    }
    double cur_pos[6];
    robotics_math::Mat2Eula(T, cur_pos);

    // 位置误差
    for (int i = 0; i < 3; i++) dxe(i, 0) = desEular[i] - cur_pos[i];

    // 姿态误差（统一Z-Y-X顺序，与旧代码一致）
    double qc[4], qd[4], inv_qc[4], delta_q[4], q_axis[4];
    robotics_math::Eul2Quat(cur_pos[5], cur_pos[4], cur_pos[3], qc);  // Z-Y-X顺序
    robotics_math::Eul2Quat(desEular[5], desEular[4], desEular[3], qd);
    robotics_math::InvQuat(qc, inv_qc);
    robotics_math::MultiQuat(qd, inv_qc, delta_q);
    robotics_math::GetQuaAxis(delta_q, q_axis);
    dxe(3, 0) = q_axis[0] * q_axis[1];
    dxe(4, 0) = q_axis[0] * q_axis[2];
    dxe(5, 0) = q_axis[0] * q_axis[3];

    // 雅可比
    double jac[6][NUM_JOINTS_PER_ARM];
    robotics_math::getJacobianGeneral(q, robot.mdh.alpha.data(), robot.mdh.a.data(), robot.mdh.d.data(), NUM_JOINTS_PER_ARM, jac);
    for (int i = 0; i < 6; i++) {
        for (int j = 0; j < NUM_JOINTS_PER_ARM; j++) {
            J(i, j) = jac[i][j];
        }
    }
}

void kinematics::worldToBase(const Robot& robot, double base_pos[6], const double world_pos[6]) {
    matrix::Matrix<double, 4, 4> T_trans, T_rot;
    T_trans.identity();
    T_rot.identity();
    T_trans(1, 3) = -robot.mdh.base_to_world[1];
    T_trans(2, 3) = -robot.mdh.base_to_world[2];
    double theta = robot.mdh.base_to_world[3];
    T_rot(1, 1) = std::cos(theta);
    T_rot(1, 2) = -std::sin(theta);
    T_rot(2, 1) = std::sin(theta);
    T_rot(2, 2) = std::cos(theta);
    matrix::Matrix<double, 4, 4> T_w2b = T_rot.transpose() * T_trans;
    matrix::Matrix<double, 4, 4> T_world;
    robotics_math::Eula2Mat(T_world, world_pos);
    matrix::Matrix<double, 4, 4> T_base = T_w2b * T_world;
    robotics_math::Mat2Eula(T_base, base_pos);
}

void kinematics::baseToWorld(const Robot& robot, double world_pos[6], const double base_pos[6]) {
    matrix::Matrix<double, 4, 4> T_trans, T_rot;
    T_trans.identity();
    T_rot.identity();
    T_trans(1, 3) = robot.mdh.base_to_world[1];
    T_trans(2, 3) = robot.mdh.base_to_world[2];
    double theta = robot.mdh.base_to_world[3];
    T_rot(1, 1) = std::cos(theta);
    T_rot(1, 2) = -std::sin(theta);
    T_rot(2, 1) = std::sin(theta);
    T_rot(2, 2) = std::cos(theta);
    matrix::Matrix<double, 4, 4> T_b2w = T_trans * T_rot;
    matrix::Matrix<double, 4, 4> T_base;
    robotics_math::Eula2Mat(T_base, base_pos);
    matrix::Matrix<double, 4, 4> T_world = T_b2w * T_base;
    robotics_math::Mat2Eula(T_world, world_pos);
}

void kinematics::addTool(const Robot& robot, double eef_pos[6], const double link7_pos[6]) {
    matrix::Matrix<double, 4, 4> T_trans, T_rot;
    T_trans.identity();
    T_rot.identity();
    T_trans(0, 3) = robot.mdh.tool[0];
    matrix::Matrix<double, 4, 4> T = T_trans * T_rot;
    matrix::Matrix<double, 4, 4> T_link7;
    robotics_math::Eula2Mat(T_link7, link7_pos);
    matrix::Matrix<double, 4, 4> T_eef = T_link7 * T;
    robotics_math::Mat2Eula(T_eef, eef_pos);
}

void kinematics::removeTool(const Robot& robot, double link7_pos[6], const double eef_pos[6]) {
    matrix::Matrix<double, 4, 4> T_trans, T_rot;
    T_trans.identity();
    T_rot.identity();
    T_trans(0, 3) = -robot.mdh.tool[0];
    matrix::Matrix<double, 4, 4> T = T_trans * T_rot;
    matrix::Matrix<double, 4, 4> T_eef;
    robotics_math::Eula2Mat(T_eef, eef_pos);
    matrix::Matrix<double, 4, 4> T_link7 = T_eef * T;
    robotics_math::Mat2Eula(T_link7, link7_pos);
}

void kinematics::show(const double cart_pos[6]) {
    std::cout << "cart_pos (x,y,z,rx,ry,rz): " << cart_pos[0] << ", " << cart_pos[1] << ", " << cart_pos[2]
              << ", " << cart_pos[3] << ", " << cart_pos[4] << ", " << cart_pos[5] << std::endl;
}

void kinematics::showjoint(const double joint[NUM_JOINTS_PER_ARM]) {
    std::cout << "joint: ";
    for (int i = 0; i < NUM_JOINTS_PER_ARM; i++) {
        std::cout << joint[i] << " ";
    }
    std::cout << std::endl;
}

void kinematics::show(const matrix::Matrix<double, 4, 4>& T) {
    std::cout << "T_matrix:" << std::endl;
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            std::cout << T(i, j) << "\t";
        }
        std::cout << std::endl;
    }
}