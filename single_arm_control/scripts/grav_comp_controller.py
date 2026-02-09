import pinocchio as pin
import numpy as np
import math  # 用于 sin, cos, math.pow 等


class GravityCompensator:
    """
    重力补偿计算器
    只负责加载模型和计算重力项，不涉及ROS话题
    支持两种计算方式：基于Pinocchio的精确计算，以及基于线性化参数的近似计算（从C++代码转换而来）
    """
    def __init__(self, urdf_string: str):
        
        # 构建 pinocchio 模型（用于精确计算）
        self.model = pin.buildModelFromXML(urdf_string)
        self.data = self.model.createData()

        # 假设你的机械臂有 6 个关节，且顺序与 URDF 一致
        # 如果关节数量或名称不同，请在这里修改
        self.nq = self.model.nq
        print(f"加载模型成功，自由度数: {self.nq}")
        self.g = -9.81
        # 可选：确认重力方向（通常是 -Z 方向）
        # 如果你的世界坐标系不同，可以在这里修改
        # self.model.gravity.linear = np.array([0, 0, -9.81])

        # 从C++代码中提取的固定参数
        self.L = [0.0962, 0.0965, 0.3465, 0.15, 0.0965, 0.185]  # L 参数
        self.beta = [-0.281818937911963, -0.316608961504547, -0.628938623679062, 0.118998250501510, -0.0235182825427395,
                     -0.220192898380960, 0.543475955741431, -0.367117250199606, 0.0102276874399998, -0.00580008163380773,
                     0.139368930309995, -0.121005108241876, 0.0843768414604314, -0.0000274978947527836, -0.0209750061732259,
                     0.257331376387291, -0.0964637762469175, 0.0336380820936370, 0.00391579503587261, 0.00675894694685360,
                     0.663042648643003, 0.220232161009426, 0.00689223035117759, -0.0778936597613123, 0.0505277187365738,
                     0.0145474517153272, 0.830335640563382, 0.174927512843074, 0.0233018202537892, 0.00421562265007968,
                     -0.0222144124572240, -0.0124892419268965, 0.00278617636357429, -0.0883153781743795, -0.0310362254559417,
                     0.00363951033664802, 0.0292516371316976, 0.0311460478783964, 0.0119936277680614, 0.00446072779170141]  # beta 参数（40维）

        self.friction_params = [1.46662181652836, 2.87362664634089, 2.86603792527201, 1.06665523043001,
                                1.58421626091361, 2.07803699751541, 0., -0.,
                                0., -0., 0., 0.]  # 摩擦参数

        # 重力加速度

    def compute_gravity_compensation_pinocchio(self, q: np.ndarray) -> np.ndarray:
        """
        计算重力补偿力矩（使用Pinocchio精确计算）
        输入:
            q: 当前关节位置 (numpy array, shape=(nq,))
        输出:
            tau_comp: 重力补偿力矩 (负的重力项)，shape=(nq,)
        """
        if len(q) != self.nq:
            raise ValueError(f"输入关节位置维度错误，期望 {self.nq}，实际 {len(q)}")

        # 计算重力项 g(q)
        pin.computeGeneralizedGravity(self.model, self.data, q)

        # 补偿力矩 = -g(q)
        tau_comp = self.data.g

        return tau_comp

    def compute_gravity_compensation_linearized(self, q: np.ndarray, q_dot: np.ndarray, grav_gain: np.ndarray = None, fric_gain: np.ndarray = None) -> np.ndarray:
        """
        计算重力补偿力矩（使用线性化参数模型，从C++代码转换而来）
        输入:
            q: 当前关节位置 (numpy array, shape=(6,))
            q_dot: 当前关节速度 (numpy array, shape=(6,))
            grav_gain: 重力补偿增益 (numpy array, shape=(6,))，默认为 [1.0]*6
            fric_gain: 摩擦补偿增益 (numpy array, shape=(6,))，默认为 [1.0]*6
        输出:
            tau: 补偿扭矩 (numpy array, shape=(6,))
        注意: 此方法计算 CCG (Coriolis + Centrifugal + Gravity) + Friction
        """
        if len(q) != 6 or len(q_dot) != 6:
            raise ValueError("输入 q 或 q_dot 维度必须为6")

        if grav_gain is None:
            grav_gain = np.ones(6)
        if fric_gain is None:
            fric_gain = np.ones(6)

        if len(grav_gain) != 6 or len(fric_gain) != 6:
            raise ValueError("grav_gain 或 fric_gain 维度必须为6")

        # 提取 q 和 q_dot
        q1, q2, q3, q4, q5, q6 = q
        qd1, qd2, qd3, qd4, qd5, qd6 = q_dot

        # 提取 L
        L1, L2, L3, L4, L5, L6 = self.L

        # 提取 beta 参数
        I1xx = 0;  I2xx = self.beta[0];  I3xx = self.beta[1];  I4xx = self.beta[2];  I5xx = self.beta[3];  I6xx = self.beta[4]
        I1xy = 0;  I2xy = self.beta[5];  I3xy = self.beta[6];  I4xy = self.beta[7];  I5xy = self.beta[8];  I6xy = self.beta[9]
        I1xz = 0;  I2xz = self.beta[10];  I3xz = self.beta[11];  I4xz = self.beta[12];  I5xz = self.beta[13];  I6xz = self.beta[14]
        I1yz = 0;  I2yz = self.beta[15];  I3yz = self.beta[16];  I4yz = self.beta[17];  I5yz = self.beta[18];  I6yz = self.beta[19]
        I1zz = self.beta[20];  I2zz = self.beta[21];  I3zz = self.beta[22];  I4zz = self.beta[23];  I5zz = self.beta[24];  I6zz = self.beta[25]
        mx1 = 0;  mx2 = self.beta[26];  mx3 = self.beta[27];  mx4 = self.beta[28];  mx5 = self.beta[29];  mx6 = self.beta[30]
        my1 = 0;  my2 = self.beta[31];  my3 = self.beta[32];  my4 = self.beta[33];  my5 = self.beta[34];  my6 = self.beta[35]
        Ia1 = 0;  Ia2 = 0;  Ia3 = self.beta[36];  Ia4 = self.beta[37];  Ia5 = self.beta[38];  Ia6 = self.beta[39]
        m1 = 0;  m2 = 0;  m3 = 0;  m4 = 0;  m5 = 0;  m6 = 0
        mz1 = 0;  mz2 = 0;  mz3 = 0;  mz4 = 0;  mz5 = 0;  mz6 = 0
        I1yy = 0;  I2yy = 0;  I3yy = 0;  I4yy = 0;  I5yy = 0;  I6yy = 0
        S2 = math.cos(q2)
        C2 = -math.sin(q2)
        S3 = math.sin(q3)
        C3 = math.cos(q3)
        S4 = math.sin(q4)
        C4 = math.cos(q4)
        S5 = math.cos(q5)
        C5 = -math.sin(q5)
        S6 = math.cos(q6)
        C6 = -math.sin(q6)
        DV331 = -math.pow(qd1, 2)
        WI12 = qd1 * S2
        WI22 = C2 * qd1
        WP12 = qd2 * WI22
        WP22 = -(qd2 * WI12)
        DV112 = -math.pow(WI12, 2)
        DV222 = -math.pow(WI22, 2)
        DV332 = -math.pow(qd2, 2)
        DV122 = WI12 * WI22
        DV132 = qd2 * WI12
        DV232 = qd2 * WI22
        U112 = DV222 + DV332
        U132 = DV132 + WP22
        U232 = DV232 - WP12
        U312 = DV132 - WP22
        U322 = DV232 + WP12
        U332 = DV112 + DV222
        VSP12 = DV331 * L2
        VP12 = -(self.g * S2) + C2 * VSP12
        VP22 = -(C2 * self.g) - S2 * VSP12
        F32 = mx2 * U312 + my2 * U322 + mz2 * U332
        PIS12 = -I2yy + I2zz
        PIS22 = I2xx - I2zz
        PIS32 = -I2xx + I2yy
        No12 = DV122 * I2xz + (-DV222 + DV332) * I2yz + DV232 * PIS12 - I2xy * U312 + I2xx * WP12
        No22 = (DV112 - DV332) * I2xz - DV122 * I2yz + DV132 * PIS22 + I2xy * U322 + I2yy * WP22
        No32 = (-DV112 + DV222) * I2xy + DV122 * PIS32 + I2yz * U132 - I2xz * U232
        WI13 = C3 * WI12 + S3 * WI22
        WI23 = -(S3 * WI12) + C3 * WI22
        W33 = qd2 + qd3
        WP13 = qd3 * WI23 + C3 * WP12 + S3 * WP22
        WP23 = -(qd3 * WI13) - S3 * WP12 + C3 * WP22
        DV113 = -math.pow(WI13, 2)
        DV223 = -math.pow(WI23, 2)
        DV333 = -math.pow(W33, 2)
        DV123 = WI13 * WI23
        DV133 = W33 * WI13
        DV233 = W33 * WI23
        U113 = DV223 + DV333
        U133 = DV133 + WP23
        U223 = DV113 + DV333
        U233 = DV233 - WP13
        U313 = DV133 - WP23
        U323 = DV233 + WP13
        U333 = DV113 + DV223
        VSP13 = L3 * U112 + VP12
        VSP23 = DV122 * L3 + VP22
        VSP33 = L3 * U312
        VP13 = C3 * VSP13 + S3 * VSP23
        VP23 = -(S3 * VSP13) + C3 * VSP23
        F13 = DV123 * my3 + mx3 * U113 + mz3 * U133 + m3 * VP13
        F23 = DV123 * mx3 + my3 * U223 + mz3 * U233 + m3 * VP23
        F33 = mx3 * U313 + my3 * U323 + mz3 * U333 + m3 * VSP33
        PIS13 = -I3yy + I3zz
        PIS23 = I3xx - I3zz
        PIS33 = -I3xx + I3yy
        No13 = DV123 * I3xz + (-DV223 + DV333) * I3yz + DV233 * PIS13 - I3xy * U313 + I3xx * WP13
        No23 = (DV113 - DV333) * I3xz - DV123 * I3yz + DV133 * PIS23 + I3xy * U323 + I3yy * WP23
        No33 = (-DV113 + DV223) * I3xy + DV123 * PIS33 + I3yz * U133 - I3xz * U233
        WI14 = C4 * WI13 + S4 * WI23
        WI24 = -(S4 * WI13) + C4 * WI23
        W34 = qd4 + W33
        WP14 = qd4 * WI24 + C4 * WP13 + S4 * WP23
        WP24 = -(qd4 * WI14) - S4 * WP13 + C4 * WP23
        DV114 = -math.pow(WI14, 2)
        DV224 = -math.pow(WI24, 2)
        DV334 = -math.pow(W34, 2)
        DV124 = WI14 * WI24
        DV134 = W34 * WI14
        DV234 = W34 * WI24
        U114 = DV224 + DV334
        U134 = DV134 + WP24
        U224 = DV114 + DV334
        U234 = DV234 - WP14
        U314 = DV134 - WP24
        U324 = DV234 + WP14
        U334 = DV114 + DV224
        VSP14 = L4 * U113 + VP13
        VSP24 = DV123 * L4 + VP23
        VSP34 = L4 * U313 + VSP33
        VP14 = C4 * VSP14 + S4 * VSP24
        VP24 = -(S4 * VSP14) + C4 * VSP24
        F14 = DV124 * my4 + mx4 * U114 + mz4 * U134 + m4 * VP14  
        F24= DV124 * mx4 + my4 * U224 + mz4 * U234 + m4*VP24
        F34=mx4*U314 + my4*U324 + mz4*U334 + m4*VSP34
        PIS14 = -I4yy + I4zz
        PIS24 = I4xx - I4zz
        PIS34 = -I4xx + I4yy
        No14 = DV124 * I4xz + (-DV224 + DV334) * I4yz + DV234 * PIS14 - I4xy * U314 + I4xx * WP14
        No24 = (DV114 - DV334) * I4xz - DV124 * I4yz + DV134 * PIS24 + I4xy * U324 + I4yy * WP24
        No34 = (-DV114 + DV224) * I4xy + DV124 * PIS34 + I4yz * U134 - I4xz * U234
        WI15 = S5 * W34 + C5 * WI14
        WI25 = C5 * W34 - S5 * WI14
        W35 = qd5 - WI24
        WP15 = qd5 * WI25 + C5 * WP14
        WP25 = -(qd5 * WI15) - S5 * WP14
        DV115 = -math.pow(WI15, 2)
        DV225 = -math.pow(WI25, 2)
        DV335 = -math.pow(W35, 2)
        DV125 = WI15 * WI25
        DV135 = W35 * WI15
        DV235 = W35 * WI25
        U115 = DV225 + DV335
        U125 = DV125 + WP24
        U135 = DV135 + WP25
        U215 = DV125 - WP24
        U225 = DV115 + DV335
        U235 = DV235 - WP15
        U315 = DV135 - WP25
        U325 = DV235 + WP15
        U335 = DV115 + DV225
        VSP15 = -(DV124 * L5) + VP14
        VSP25 = -(L5 * U224) + VP24
        VSP35 = -(L5 * U324) + VSP34
        VP15 = C5 * VSP15 + S5 * VSP35
        VP25 = -(S5 * VSP15) + C5 * VSP35
        F15 = mx5 * U115 + my5 * U125 + mz5 * U135 + m5 * VP15
        F25 = mx5 * U215 + my5 * U225 + mz5 * U235 + m5 * VP25
        F35 = mx5 * U315 + my5 * U325 + mz5 * U335 - m5 * VSP25
        PIS15 = -I5yy + I5zz
        PIS25 = I5xx - I5zz
        PIS35 = -I5xx + I5yy
        No15 = (-DV225 + DV335) * I5yz + DV235 * PIS15 + I5xz * U215 - I5xy * U315 + I5xx * WP15
        No25 = (DV115 - DV335) * I5xz + DV135 * PIS25 - I5yz * U125 + I5xy * U325 + I5yy * WP25
        No35 = (-DV115 + DV225) * I5xy + DV125 * PIS35 + I5yz * U135 - I5xz * U235 - I5zz * WP24
        WI16 = S6 * W35 + C6 * WI15
        WI26 = C6 * W35 - S6 * WI15
        W36 = qd6 - WI25
        WP16 = qd6 * WI26 + C6 * WP15 - S6 * WP24
        WP26 = -(qd6 * WI16) - S6 * WP15 - C6 * WP24
        DV116 = -math.pow(WI16, 2)
        DV226 = -math.pow(WI26, 2)
        DV336 = -math.pow(W36, 2)
        DV126 = WI16 * WI26
        DV136 = W36 * WI16
        DV236 = W36 * WI26
        U116 = DV226 + DV336
        U126 = DV126 + WP25
        U136 = DV136 + WP26
        U216 = DV126 - WP25
        U226 = DV116 + DV336
        U236 = DV236 - WP16
        U316 = DV136 - WP26
        U326 = DV236 + WP16
        U336 = DV116 + DV226
        VSP16 = -(L6 * U125) + VP15
        VSP26 = -(L6 * U225) + VP25
        VSP36 = -(L6 * U325) - VSP25
        VP16 = C6 * VSP16 + S6 * VSP36
        VP26 = -(S6 * VSP16) + C6 * VSP36
        F16 = mx6 * U116 + my6 * U126 + mz6 * U136 + m6 * VP16
        F26 = mx6 * U216 + my6 * U226 + mz6 * U236 + m6 * VP26
        F36 = mx6 * U316 + my6 * U326 + mz6 * U336 - m6 * VSP26
        PIS16 = -I6yy + I6zz
        PIS26 = I6xx - I6zz
        PIS36 = -I6xx + I6yy
        No16 = (-DV226 + DV336) * I6yz + DV236 * PIS16 + I6xz * U216 - I6xy * U316 + I6xx * WP16
        No26 = (DV116 - DV336) * I6xz + DV136 * PIS26 - I6yz * U126 + I6xy * U326 + I6yy * WP26
        No36 = (-DV116 + DV226) * I6xy + DV126 * PIS36 + I6yz * U136 - I6xz * U236 - I6zz * WP25
        N16 = No16 - mz6 * VP26 - my6 * VSP26
        N26 = No26 + mz6 * VP16 + mx6 * VSP26
        N36 = No36 - my6 * VP16 + mx6 * VP26
        FDI16 = C6 * F16 - F26 * S6
        FDI36 = C6 * F26 + F16 * S6
        E15 = F15 + FDI16
        E25 = F25 - F36
        E35 = F35 + FDI36
        N15 = -(FDI36 * L6) + C6 * N16 + No15 - N26 * S6 - mz5 * VP25 - my5 * VSP25
        N25 = -N36 + No25 + mz5 * VP15 + mx5 * VSP25
        N35 = FDI16 * L6 + C6 * N26 + No35 + N16 * S6 - my5 * VP15 + mx5 * VP25
        FDI15 = C5 * E15 - E25 * S5
        FDI35 = C5 * E25 + E15 * S5
        E14 = F14 + FDI15
        E24 = -E35 + F24  
        E34 = F34 + FDI35  
        N14 = -(FDI35 * L5) + C5 * N15 + No14 - N25 * S5 - mz4 * VP24 + my4 * VSP34
        N24 = -N35 + No24 + mz4 * VP14 - mx4 * VSP34
        N34 = FDI15 * L5 + C5 * N25 + No34 + N15 * S5 - my4 * VP14 + mx4 * VP24
        FDI14 = C4 * E14 - E24 * S4
        FDI24 = C4 * E24 + E14 * S4
        E13 = F13 + FDI14
        E23 = F23 + FDI24
        E33 = E34 + F33
        N13 = C4 * N14 + No13 - N24 * S4 - mz3 * VP23 + my3 * VSP33
        N23 = -(E34 * L4) + C4 * N24 + No23 + N14 * S4 + mz3 * VP13 - mx3 * VSP33
        N33 = FDI24 * L4 + N34 + No33 - my3 * VP13 + mx3 * VP23
        FDI23 = C3 * E23 + E13 * S3
        E32 = E33 + F32
        N12 = C3 * N13 + No12 - N23 * S3 - mz2 * VP22
        N22 = -(E33 * L3) + C3 * N23 + No22 + N13 * S3 + mz2 * VP12
        N32 = FDI23 * L3 + N33 + No32 - my2 * VP12 + mx2 * VP22
        N31 = -(E32 * L2) + C2 * N22 + N12 * S2
        H1 = N31
        H2 = N32
        H3 = N33
        H4 = N34
        H5 = N35
        H6 = N36
        tau_ccg = {H1, H2, H3, H4, H5, H6}

        tau_ccg = np.array([H1, H2, H3, H4, H5, H6])

        # 应用 gain 和摩擦
        tau = np.zeros(6)
        for i in range(6):
            tau[i] = grav_gain[i] * tau_ccg[i]
            qdi = q_dot[i]
            sign_qdi = 1.0 if qdi > 0.0 else (-1.0 if qdi < 0.0 else 0.0)
            fc = self.friction_params[2 * i]      # 库仑摩擦
            fv = self.friction_params[2 * i + 1]  # 粘性摩擦
            tau[i] += fric_gain[i] * (fv * qdi + fc * sign_qdi)

        return tau

    def get_joint_names(self):
        """返回模型中的关节名称列表（调试用）"""
        return [joint.name for joint in self.model.joints if joint.name != "universe"]


# 示例用法（非ROS环境测试）
if __name__ == "__main__":
    # 假设你有 URDF 字符串
    # 在实际使用中，这个字符串应该从 ROS 参数 /robot_description 获得
    import sys
    if len(sys.argv) < 2:
        print("用法: python gravity_comp_controller.py <path_to_urdf>")
        sys.exit(1)

    with open(sys.argv[1], 'r') as f:
        urdf_str = f.read()

    compensator = GravityCompensator(urdf_str)

    # 测试 Pinocchio 方法
    q_test = np.zeros(compensator.nq)           # 全零位
    tau_pin = compensator.compute_gravity_compensation_pinocchio(q_test)
    print("q =", q_test)
    print("重力补偿力矩 (Pinocchio):", tau_pin)

    q_test[0] = np.deg2rad(30)                  # joint1 转 30 度
    tau_pin = compensator.compute_gravity_compensation_pinocchio(q_test)
    print("\nq =", q_test)
    print("重力补偿力矩 (Pinocchio):", tau_pin)

    # 测试线性化方法（需提供 q_dot）
    q_dot_test = np.zeros(6)
    tau_lin = compensator.compute_gravity_compensation_linearized(q_test[:6], q_dot_test)  # 假设 nq=6
    print("\n线性化重力补偿力矩 (包括CCG + Friction):", tau_lin)