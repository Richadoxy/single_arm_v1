// ccg_controller.cpp
// 这个文件实现了一个CCGController类，用于计算单臂机器人的Coriolis、Centrifugal和Gravity补偿扭矩（tau）。
// beta参数已辨识好，需要在此硬编码为固定值（请替换下面的占位符为实际辨识值）。
// L参数也硬编码为固定值，根据提供的文档。
// 输入：q (关节位置，6维向量)，q_dot (关节速度，6维向量)
// 输出：tau (扭矩，6维向量)，即 ccg = C(q, q_dot) * q_dot + G(q)
// 先不集成ROS2，仅作为独立类。

#include <vector>
#include <cmath>  // 用于 sin, cos, pow 等
#include "ccg_controller.h"

CCGController::CCGController() {
    // 初始化 L 参数，从文档中提取
    L = {0.0962, 0.0965, 0.3465, 0.15, 0.0965, 0.185};

    // 初始化 beta 参数（40维），请替换为实际辨识值
    // 示例：beta = {value1, value2, ..., value40};
    // 如果未提供，默认为0（测试用），实际使用时必须更新
    beta = {-0.281818937911963, -0.316608961504547, -0.628938623679062, 0.118998250501510, -0.0235182825427395,
            -0.220192898380960, 0.543475955741431, -0.367117250199606, 0.0102276874399998, -0.00580008163380773, 
            0.139368930309995, -0.121005108241876, 0.0843768414604314, -0.0000274978947527836, -0.0209750061732259, 
            0.257331376387291, -0.0964637762469175, 0.0336380820936370, 0.00391579503587261, 0.00675894694685360, 
            0.663042648643003, 0.220232161009426, 0.00689223035117759, -0.0778936597613123, 0.0505277187365738, 
            0.0145474517153272, 0.830335640563382, 0.174927512843074, 0.0233018202537892, 0.00421562265007968, 
            -0.0222144124572240, -0.0124892419268965, 0.00278617636357429, -0.0883153781743795, -0.0310362254559417,
            0.00363951033664802, 0.0292516371316976, 0.0311460478783964, 0.0119936277680614, 0.00446072779170141};
}

    // 计算 tau = CCG (Coriolis + Centrifugal + Gravity)
void CCGController::computeTau(const std::vector<double>& q, const std::vector<double>& q_dot, std::vector<double>& tau) {
    if (q.size() != 6 || q_dot.size() != 6) {
        // 错误处理：输入维度不对
        tau = std::vector<double>(6, 0.0);
        return;
    }

    // 提取 q 和 q_dot
    double q1 = q[0];
    double q2 = q[1];
    double q3 = q[2];
    double q4 = q[3];
    double q5 = q[4];
    double q6 = q[5];

    double qd1 = q_dot[0];
    double qd2 = q_dot[1];
    double qd3 = q_dot[2];
    double qd4 = q_dot[3];
    double qd5 = q_dot[4];
    double qd6 = q_dot[5];

    // 提取 L
    double L1 = L[0];
    double L2 = L[1];
    double L3 = L[2];
    double L4 = L[3];
    double L5 = L[4];
    double L6 = L[5];

    // 重力加速度
    double g = -9.81;

    // 提取 beta 参数
    double I1xx = 0; double I2xx = beta[0]; double I3xx = beta[1]; double I4xx = beta[2]; double I5xx = beta[3]; double I6xx = beta[4];
    double I1xy = 0; double I2xy = beta[5]; double I3xy = beta[6]; double I4xy = beta[7]; double I5xy = beta[8]; double I6xy = beta[9];
    double I1xz = 0; double I2xz = beta[10]; double I3xz = beta[11]; double I4xz = beta[12]; double I5xz = beta[13]; double I6xz = beta[14];
    double I1yz = 0; double I2yz = beta[15]; double I3yz = beta[16]; double I4yz = beta[17]; double I5yz = beta[18]; double I6yz = beta[19];
    double I1zz = beta[20]; double I2zz = beta[21]; double I3zz = beta[22]; double I4zz = beta[23]; double I5zz = beta[24]; double I6zz = beta[25];
    double mx1 = 0; double mx2 = beta[26]; double mx3 = beta[27]; double mx4 = beta[28]; double mx5 = beta[29]; double mx6 = beta[30];
    double my1 = 0; double my2 = beta[31]; double my3 = beta[32]; double my4 = beta[33]; double my5 = beta[34]; double my6 = beta[35];
    double Ia1 = 0; double Ia2 = 0; double Ia3 = beta[36]; double Ia4 = beta[37]; double Ia5 = beta[38]; double Ia6 = beta[39];
    double m1 = 0; double m2 = 0; double m3 = 0; double m4 = 0; double m5 = 0; double m6 = 0;
    double mz1 = 0; double mz2 = 0; double mz3 = 0; double mz4 = 0; double mz5 = 0; double mz6 = 0;
    double I1yy = 0; double I2yy = 0; double I3yy = 0; double I4yy = 0; double I5yy = 0; double I6yy = 0;

    // 计算变量（直接翻译 MATLAB 代码）
    double S2 = cos(q2);
    double C2 = -sin(q2);
    double S3 = sin(q3);
    double C3 = cos(q3);
    double S4 = sin(q4);
    double C4 = cos(q4);
    double S5 = cos(q5);
    double C5 = -sin(q5);
    double S6 = cos(q6);
    double C6 = -sin(q6);
    double DV331 = -pow(qd1, 2);
    double WI12 = qd1 * S2;
    double WI22 = C2 * qd1;
    double WP12 = qd2 * WI22;
    double WP22 = -(qd2 * WI12);
    double DV112 = -pow(WI12, 2);
    double DV222 = -pow(WI22, 2);
    double DV332 = -pow(qd2, 2);
    double DV122 = WI12 * WI22;
    double DV132 = qd2 * WI12;
    double DV232 = qd2 * WI22;
    double U112 = DV222 + DV332;
    double U132 = DV132 + WP22;
    double U232 = DV232 - WP12;
    double U312 = DV132 - WP22;
    double U322 = DV232 + WP12;
    double U332 = DV112 + DV222;
    double VSP12 = DV331 * L2;
    double VP12 = -(g * S2) + C2 * VSP12;
    double VP22 = -(C2 * g) - S2 * VSP12;
    double F32 = mx2 * U312 + my2 * U322 + mz2 * U332;
    double PIS12 = -I2yy + I2zz;
    double PIS22 = I2xx - I2zz;
    double PIS32 = -I2xx + I2yy;
    double No12 = DV122 * I2xz + (-DV222 + DV332) * I2yz + DV232 * PIS12 - I2xy * U312 + I2xx * WP12;
    double No22 = (DV112 - DV332) * I2xz - DV122 * I2yz + DV132 * PIS22 + I2xy * U322 + I2yy * WP22;
    double No32 = (-DV112 + DV222) * I2xy + DV122 * PIS32 + I2yz * U132 - I2xz * U232;
    double WI13 = C3 * WI12 + S3 * WI22;
    double WI23 = -(S3 * WI12) + C3 * WI22;
    double W33 = qd2 + qd3;
    double WP13 = qd3 * WI23 + C3 * WP12 + S3 * WP22;
    double WP23 = -(qd3 * WI13) - S3 * WP12 + C3 * WP22;
    double DV113 = -pow(WI13, 2);
    double DV223 = -pow(WI23, 2);
    double DV333 = -pow(W33, 2);
    double DV123 = WI13 * WI23;
    double DV133 = W33 * WI13;
    double DV233 = W33 * WI23;
    double U113 = DV223 + DV333;
    double U133 = DV133 + WP23;
    double U223 = DV113 + DV333;
    double U233 = DV233 - WP13;
    double U313 = DV133 - WP23;
    double U323 = DV233 + WP13;
    double U333 = DV113 + DV223;
    double VSP13 = L3 * U112 + VP12;
    double VSP23 = DV122 * L3 + VP22;
    double VSP33 = L3 * U312;
    double VP13 = C3 * VSP13 + S3 * VSP23;
    double VP23 = -(S3 * VSP13) + C3 * VSP23;
    double F13 = DV123 * my3 + mx3 * U113 + mz3 * U133 + m3 * VP13;
    double F23 = DV123 * mx3 + my3 * U223 + mz3 * U233 + m3 * VP23;
    double F33 = mx3 * U313 + my3 * U323 + mz3 * U333 + m3 * VSP33;
    double PIS13 = -I3yy + I3zz;
    double PIS23 = I3xx - I3zz;
    double PIS33 = -I3xx + I3yy;
    double No13 = DV123 * I3xz + (-DV223 + DV333) * I3yz + DV233 * PIS13 - I3xy * U313 + I3xx * WP13;
    double No23 = (DV113 - DV333) * I3xz - DV123 * I3yz + DV133 * PIS23 + I3xy * U323 + I3yy * WP23;
    double No33 = (-DV113 + DV223) * I3xy + DV123 * PIS33 + I3yz * U133 - I3xz * U233;
    double WI14 = C4 * WI13 + S4 * WI23;
    double WI24 = -(S4 * WI13) + C4 * WI23;
    double W34 = qd4 + W33;
    double WP14 = qd4 * WI24 + C4 * WP13 + S4 * WP23;
    double WP24 = -(qd4 * WI14) - S4 * WP13 + C4 * WP23;
    double DV114 = -pow(WI14, 2);
    double DV224 = -pow(WI24, 2);
    double DV334 = -pow(W34, 2);
    double DV124 = WI14 * WI24;
    double DV134 = W34 * WI14;
    double DV234 = W34 * WI24;
    double U114 = DV224 + DV334;
    double U134 = DV134 + WP24;
    double U224 = DV114 + DV334;
    double U234 = DV234 - WP14;
    double U314 = DV134 - WP24;
    double U324 = DV234 + WP14;
    double U334 = DV114 + DV224;
    double VSP14 = L4 * U113 + VP13;
    double VSP24 = DV123 * L4 + VP23;
    double VSP34 = L4 * U313 + VSP33;
    double VP14 = C4 * VSP14 + S4 * VSP24;
    double VP24 = -(S4 * VSP14) + C4 * VSP24;
    double F14 = DV124 * my4 + mx4 * U114 + mz4 * U134 + m4 * VP14;  
    double F24= DV124 * mx4 + my4 * U224 + mz4 * U234 + m4*VP24;
    double F34=mx4*U314 + my4*U324 + mz4*U334 + m4*VSP34;
    double PIS14 = -I4yy + I4zz;
    double PIS24 = I4xx - I4zz;
    double PIS34 = -I4xx + I4yy;
    double No14 = DV124 * I4xz + (-DV224 + DV334) * I4yz + DV234 * PIS14 - I4xy * U314 + I4xx * WP14;
    double No24 = (DV114 - DV334) * I4xz - DV124 * I4yz + DV134 * PIS24 + I4xy * U324 + I4yy * WP24;
    double No34 = (-DV114 + DV224) * I4xy + DV124 * PIS34 + I4yz * U134 - I4xz * U234;
    double WI15 = S5 * W34 + C5 * WI14;
    double WI25 = C5 * W34 - S5 * WI14;
    double W35 = qd5 - WI24;
    double WP15 = qd5 * WI25 + C5 * WP14;
    double WP25 = -(qd5 * WI15) - S5 * WP14;
    double DV115 = -pow(WI15, 2);
    double DV225 = -pow(WI25, 2);
    double DV335 = -pow(W35, 2);
    double DV125 = WI15 * WI25;
    double DV135 = W35 * WI15;
    double DV235 = W35 * WI25;
    double U115 = DV225 + DV335;
    double U125 = DV125 + WP24;
    double U135 = DV135 + WP25;
    double U215 = DV125 - WP24;
    double U225 = DV115 + DV335;
    double U235 = DV235 - WP15;
    double U315 = DV135 - WP25;
    double U325 = DV235 + WP15;
    double U335 = DV115 + DV225;
    double VSP15 = -(DV124 * L5) + VP14;
    double VSP25 = -(L5 * U224) + VP24;
    double VSP35 = -(L5 * U324) + VSP34;
    double VP15 = C5 * VSP15 + S5 * VSP35;
    double VP25 = -(S5 * VSP15) + C5 * VSP35;
    double F15 = mx5 * U115 + my5 * U125 + mz5 * U135 + m5 * VP15;
    double F25 = mx5 * U215 + my5 * U225 + mz5 * U235 + m5 * VP25;
    double F35 = mx5 * U315 + my5 * U325 + mz5 * U335 - m5 * VSP25;
    double PIS15 = -I5yy + I5zz;
    double PIS25 = I5xx - I5zz;
    double PIS35 = -I5xx + I5yy;
    double No15 = (-DV225 + DV335) * I5yz + DV235 * PIS15 + I5xz * U215 - I5xy * U315 + I5xx * WP15;
    double No25 = (DV115 - DV335) * I5xz + DV135 * PIS25 - I5yz * U125 + I5xy * U325 + I5yy * WP25;
    double No35 = (-DV115 + DV225) * I5xy + DV125 * PIS35 + I5yz * U135 - I5xz * U235 - I5zz * WP24;
    double WI16 = S6 * W35 + C6 * WI15;
    double WI26 = C6 * W35 - S6 * WI15;
    double W36 = qd6 - WI25;
    double WP16 = qd6 * WI26 + C6 * WP15 - S6 * WP24;
    double WP26 = -(qd6 * WI16) - S6 * WP15 - C6 * WP24;
    double DV116 = -pow(WI16, 2);
    double DV226 = -pow(WI26, 2);
    double DV336 = -pow(W36, 2);
    double DV126 = WI16 * WI26;
    double DV136 = W36 * WI16;
    double DV236 = W36 * WI26;
    double U116 = DV226 + DV336;
    double U126 = DV126 + WP25;
    double U136 = DV136 + WP26;
    double U216 = DV126 - WP25;
    double U226 = DV116 + DV336;
    double U236 = DV236 - WP16;
    double U316 = DV136 - WP26;
    double U326 = DV236 + WP16;
    double U336 = DV116 + DV226;
    double VSP16 = -(L6 * U125) + VP15;
    double VSP26 = -(L6 * U225) + VP25;
    double VSP36 = -(L6 * U325) - VSP25;
    double VP16 = C6 * VSP16 + S6 * VSP36;
    double VP26 = -(S6 * VSP16) + C6 * VSP36;
    double F16 = mx6 * U116 + my6 * U126 + mz6 * U136 + m6 * VP16;
    double F26 = mx6 * U216 + my6 * U226 + mz6 * U236 + m6 * VP26;
    double F36 = mx6 * U316 + my6 * U326 + mz6 * U336 - m6 * VSP26;
    double PIS16 = -I6yy + I6zz;
    double PIS26 = I6xx - I6zz;
    double PIS36 = -I6xx + I6yy;
    double No16 = (-DV226 + DV336) * I6yz + DV236 * PIS16 + I6xz * U216 - I6xy * U316 + I6xx * WP16;
    double No26 = (DV116 - DV336) * I6xz + DV136 * PIS26 - I6yz * U126 + I6xy * U326 + I6yy * WP26;
    double No36 = (-DV116 + DV226) * I6xy + DV126 * PIS36 + I6yz * U136 - I6xz * U236 - I6zz * WP25;
    double N16 = No16 - mz6 * VP26 - my6 * VSP26;
    double N26 = No26 + mz6 * VP16 + mx6 * VSP26;
    double N36 = No36 - my6 * VP16 + mx6 * VP26;
    double FDI16 = C6 * F16 - F26 * S6;
    double FDI36 = C6 * F26 + F16 * S6;
    double E15 = F15 + FDI16;
    double E25 = F25 - F36;
    double E35 = F35 + FDI36;
    double N15 = -(FDI36 * L6) + C6 * N16 + No15 - N26 * S6 - mz5 * VP25 - my5 * VSP25;
    double N25 = -N36 + No25 + mz5 * VP15 + mx5 * VSP25;
    double N35 = FDI16 * L6 + C6 * N26 + No35 + N16 * S6 - my5 * VP15 + mx5 * VP25;
    double FDI15 = C5 * E15 - E25 * S5;
    double FDI35 = C5 * E25 + E15 * S5;
    double E14 = F14 + FDI15;
    double E24 = -E35 + F24;  
    double E34 = F34 + FDI35;  // 类似，F34需定义
    double N14 = -(FDI35 * L5) + C5 * N15 + No14 - N25 * S5 - mz4 * VP24 + my4 * VSP34;
    double N24 = -N35 + No24 + mz4 * VP14 - mx4 * VSP34;
    double N34 = FDI15 * L5 + C5 * N25 + No34 + N15 * S5 - my4 * VP14 + mx4 * VP24;
    double FDI14 = C4 * E14 - E24 * S4;
    double FDI24 = C4 * E24 + E14 * S4;
    double E13 = F13 + FDI14;
    double E23 = F23 + FDI24;
    double E33 = E34 + F33;
    double N13 = C4 * N14 + No13 - N24 * S4 - mz3 * VP23 + my3 * VSP33;
    double N23 = -(E34 * L4) + C4 * N24 + No23 + N14 * S4 + mz3 * VP13 - mx3 * VSP33;
    double N33 = FDI24 * L4 + N34 + No33 - my3 * VP13 + mx3 * VP23;
    double FDI23 = C3 * E23 + E13 * S3;
    double E32 = E33 + F32;
    double N12 = C3 * N13 + No12 - N23 * S3 - mz2 * VP22;
    double N22 = -(E33 * L3) + C3 * N23 + No22 + N13 * S3 + mz2 * VP12;
    double N32 = FDI23 * L3 + N33 + No32 - my2 * VP12 + mx2 * VP22;
    double N31 = -(E32 * L2) + C2 * N22 + N12 * S2;
    double H1 = N31;
    double H2 = N32;
    double H3 = N33;
    double H4 = N34;
    double H5 = N35;
    double H6 = N36;

    // 输出 tau
    tau = {H1, H2, H3, H4, H5, H6};
}

