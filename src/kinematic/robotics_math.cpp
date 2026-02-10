/*
 * @Author:
 * @Date: 
 * @LastEditors: 
 * @LastEditTime: 
 * @FilePath: /ros2_demo/src/gac_arm_cpp/src/
 * @Description: 
 * @错误码200-299
 * @ErrorNum: 200-半径参数输入错误，请输入正值
 * @ErrorNum: 201-圆心和球心距离过大，无解
 * @ErrorNum: 202-圆心和球心距离过小，无解
 * @ErrorNum: 203-中间变量为负值，开根号后无实数解
 * @ErrorNum: 204-总质量为0，请检查输入质量参数
 */

 #include "robotics_math.h"
 #include "SquareMatrix.hpp"
 #include "Vector.hpp"
 
 void robotics_math::getTransformationMatrixMDH(double theta, double alpha, double a, double d, matrix::SquareMatrix<double, 4>& T)
 {
	 //theta, a, alpha, d为指定关节的DH参数
	 double c_alpha = cos(alpha);
	 double s_alpha = sin(alpha);
	 double c_theta = cos(theta);
	 double s_theta = sin(theta);
	 T(0, 0) = c_theta;
	 T(0, 1) = -s_theta;
	 T(0, 2) = 0;
	 T(0, 3) = a;
	 T(1, 0) = s_theta * c_alpha;
	 T(1, 1) = c_theta * c_alpha;
	 T(1, 2) = -s_alpha;
	 T(1, 3) = -s_alpha * d;
	 T(2, 0) = s_theta * s_alpha;
	 T(2, 1) = c_theta * s_alpha;
	 T(2, 2) = c_alpha;
	 T(2, 3) = c_alpha * d;
	 T(3, 0) = 0;
	 T(3, 1) = 0;
	 T(3, 2) = 0;
	 T(3, 3) = 1;
 }

	void robotics_math::getTransformationMatrixMDHdot(double theta, double alpha, double a, double d, matrix::SquareMatrix<double, 4>& T)
 {
	 //theta, a, alpha, d为指定关节的DH参数
	 double c_alpha = cos(alpha);
	 double s_alpha = sin(alpha);
	 double c_theta = cos(theta);
	 double s_theta = sin(theta);
	 T(0, 0) = -s_theta;
	 T(0, 1) = -c_theta;
	 T(0, 2) = 0;
	 T(0, 3) = 0;
	 T(1, 0) = c_theta * c_alpha;
	 T(1, 1) = -s_theta * c_alpha;
	 T(1, 2) = 0;
	 T(1, 3) = 0;
	 T(2, 0) = c_theta * s_alpha;
	 T(2, 1) = -s_theta * s_alpha;
	 T(2, 2) = 0;
	 T(2, 3) = 0;
	 T(3, 0) = 0;
	 T(3, 1) = 0;
	 T(3, 2) = 0;
	 T(3, 3) = 0;
 }
 void robotics_math::getRotationMatrixMDH(double theta, double alpha, matrix::SquareMatrix<double, 3>& R)
 {
	 //theta, a, alpha, d为指定关节的DH参数
	 double c_alpha = cos(alpha);
	 double s_alpha = sin(alpha);
	 double c_theta = cos(theta);
	 double s_theta = sin(theta);
	 R(0, 0) = c_theta;
	 R(0, 1) = -s_theta;
	 R(0, 2) = 0;
	 R(1, 0) = s_theta * c_alpha;
	 R(1, 1) = c_theta * c_alpha;
	 R(1, 2) = -s_alpha;
	 R(2, 0) = s_theta * s_alpha;
	 R(2, 1) = c_theta * s_alpha;
	 R(2, 2) = c_alpha;
 
 }
 
 int robotics_math::getJacobianGeneral(const double* q_in,const double* alpha, const double* a,const double* d, const int dim, double Jacob[6][NUM_JOINTS_PER_ARM])
 {
	 double Z[NUM_JOINTS_PER_ARM][3]{};
	 double P[NUM_JOINTS_PER_ARM][3]{};
	 double Jv[NUM_JOINTS_PER_ARM][3]{};
	 double Jw[NUM_JOINTS_PER_ARM][3]{};
 
	 matrix::Matrix<double, 4, 4> T;
	 T.identity();
	 matrix::SquareMatrix<double, 4> Ti;
	 for (int i = 0; i < dim; i++)
	 {
		 robotics_math::getTransformationMatrixMDH(q_in[i], alpha[i], a[i], d[i], Ti);
 
		 T = T * Ti;
		 for (int j = 0; j < 3; j++)
		 {
			 Z[i][j] = T(j, 2);
			 P[i][j] = T(j, 3);
		 }
	 }
 
	 double r[3]{};
	 double w[3]{};//中间变量
 
	 for (int i = 0; i < dim; i++)
	 {
		 for (int j = 0; j < 3; j++)
		 {
			 Jw[i][j] = Z[i][j];
			 r[j] = P[dim - 1][j] - P[i][j];
			 w[j] = Jw[i][j];
		 }
		 robotics_math::arrayCross(w, r, w);
		 for (int j = 0; j < 3; j++)
		 {
			 Jv[i][j] = w[j];
		 }
	 }
 
	 //雅可比矩阵赋值
	 //Jacob.setIdentity();
	 // for (int i = 0; i < 6; i++)
	 // {
	 // 	for (int j = 0; j < dim; j++)
	 // 	{
	 // 		Jacob[i][j] = 0;
	 // 	}
	 // }
	 for (int i = 0; i < 3; i++)
	 {
		 for (int j = 0; j < dim; j++)
		 {
			 Jacob[i][j] = Jv[j][i];
			 Jacob[i + 3][j] = Jw[j][i];
		 }
	 }
 
	 return 0;
 }
 
 //绕动系ZYX/固定系XYZ
 void robotics_math::Eula2Mat(matrix::Matrix<double, 4, 4>& mat, const double* P)
 {
	 mat.identity();
	 mat(0, 3) = P[0];
	 mat(1, 3) = P[1];
	 mat(2, 3) = P[2];
	 double th1, th2, th3;
	 th1 = P[3];
	 th2 = P[4];
	 th3 = P[5];
	 double c1 = cos(th1); double s1 = sin(th1);
	 double c2 = cos(th2); double s2 = sin(th2);
	 double c3 = cos(th3); double s3 = sin(th3);
	 mat(0, 0) = c2 * c3;
	 mat(0, 1) = s1 * s2 * c3 - c1 * s3;
	 mat(0, 2) = c1 * s2 * c3 + s1 * s3;
	 mat(1, 0) = c2 * s3;
	 mat(1, 1) = s1 * s2 * s3 + c1 * c3;
	 mat(1, 2) = c1 * s2 * s3 - s1 * c3;
	 mat(2, 0) = -s2;
	 mat(2, 1) = s1 * c2;
	 mat(2, 2) = c1 * c2;
 }
 
 //绕动系ZYX/固定系XYZ
 void robotics_math::Eula2Mat(matrix::Matrix<double, 4, 4>& mat, const matrix::Matrix<double, 1, 6>& P)
 {
	 mat.identity();
	 mat(0, 3) = P(0, 0);
	 mat(1, 3) = P(0, 1);
	 mat(2, 3) = P(0, 2);
	 double th1, th2, th3;
	 th1 = P(0, 3);
	 th2 = P(0, 4);
	 th3 = P(0, 5);
	 double c1 = cos(th1); double s1 = sin(th1);
	 double c2 = cos(th2); double s2 = sin(th2);
	 double c3 = cos(th3); double s3 = sin(th3);
	 mat(0, 0) = c2 * c3;
	 mat(0, 1) = s1 * s2 * c3 - c1 * s3;
	 mat(0, 2) = c1 * s2 * c3 + s1 * s3;
	 mat(1, 0) = c2 * s3;
	 mat(1, 1) = s1 * s2 * s3 + c1 * c3;
	 mat(1, 2) = c1 * s2 * s3 - s1 * c3;
	 mat(2, 0) = -s2;
	 mat(2, 1) = s1 * c2;
	 mat(2, 2) = c1 * c2;
 }
 
 // 动坐标系ZYX，等同于固定坐标系XYZ
 void robotics_math::Mat2Eula(const matrix::Matrix<double, 4, 4>& mat, double* P)
 {
	 ////PX PY PZ
	 //P[0] = mat(0, 3);
	 //P[1] = mat(1, 3);
	 //P[2] = mat(2, 3);
	 ////RX RY RZ
	 //double r11, r12, r13, r21, r22, r23, r31, r32, r33, RX, RY, RZ;
	 //r11 = mat(0, 0); r12 = mat(0, 1); r13 = mat(0, 2);
	 //r21 = mat(1, 0); r22 = mat(1, 1); r23 = mat(1, 2);
	 //r31 = mat(2, 0); r32 = mat(2, 1); r33 = mat(2, 2);
	 //if (r11 != 0)
	 //{
	 //	RY = atan2(-r31, sqrt(r11*r11 + r21 * r21));
	 //	RX = atan2(r32, r33);
	 //	RZ = atan2(r21, r11);
	 //}
	 //else if (r31 < -0)
	 //{
	 //	RY = PI / 2;
	 //	RX = atan2(r12, r22);
	 //	RZ = 0;
	 //}
	 //else
	 //{
	 //	RY = -PI / 2;
	 //	RX = atan2(-r12, r22);
	 //	RZ = 0;
	 //}
	 //P[3] = RX;
	 //P[4] = RY;
	 //P[5] = RZ;
	 //PX PY PZ
	 P[0] = mat(0, 3);
	 P[1] = mat(1, 3);
	 P[2] = mat(2, 3);
	 //RX RY RZ
	 double sy;
	 sy = sqrt(mat(0, 0) * mat(0, 0) + mat(1, 0) * mat(1, 0));
	 bool singular = (sy < ZERO_VALUE);
	 double  RX, RY, RZ;
 
	 if (!singular)
	 {
		 RY = atan2(-mat(2, 0), sy);
		 if (abs(abs(RY) - PI / 2) < 0.0001)
		 {
			 if (mat(2, 0) < -0)
			 {
				 RX = atan2(mat(0, 1), mat(1, 1));
				 RZ = 0;
			 }
			 else
			 {
				 RX = atan2(-mat(0, 1), mat(1, 1));
				 RZ = 0;
			 }
		 }
		 else
		 {
			 RX = atan2(mat(2, 1), mat(2, 2));
			 RZ = atan2(mat(1, 0), mat(0, 0));
		 }
	 }
	 else if (mat(2, 0) < -0)
	 {
		 RY = atan2(-mat(2, 0), sy);
		 RX = atan2(mat(0, 1), mat(1, 1));
		 RZ = 0;
	 }
	 else
	 {
		 RY = atan2(-mat(2, 0), sy);
		 RX = atan2(-mat(0, 1), mat(1, 1));
		 RZ = 0;
	 }
	 P[3] = RX;
	 P[4] = RY;
	 P[5] = RZ;
 }
 
 //绕动系ZYX/固定系XYZ
 void robotics_math::Mat2Eula(const matrix::Matrix<double, 4, 4>& mat, matrix::Matrix<double, 1, 6>& P)
 {
	 //PX PY PZ
	 P(0, 0) = mat(0, 3);
	 P(0, 1) = mat(1, 3);
	 P(0, 2) = mat(2, 3);
	 //RX RY RZ
	 double r11, r12, r13, r21, r22, r23, r31, r32, r33, RX, RY, RZ;
	 r11 = mat(0, 0); r12 = mat(0, 1); r13 = mat(0, 2);
	 r21 = mat(1, 0); r22 = mat(1, 1); r23 = mat(1, 2);
	 r31 = mat(2, 0); r32 = mat(2, 1); r33 = mat(2, 2);
	 if (r11 != 0)
	 {
		 RY = atan2(-r31, sqrt(r11 * r11 + r21 * r21));
		 RX = atan2(r32, r33);
		 RZ = atan2(r21, r11);
	 }
	 else if (r31 < -0)
	 {
		 RY = PI / 2;
		 RX = atan2(r12, r22);
		 RZ = 0;
	 }
	 else
	 {
		 RY = -PI / 2;
		 RX = atan2(-r12, r22);
		 RZ = 0;
	 }
	 P(0, 3) = RX;
	 P(0, 4) = RY;
	 P(0, 5) = RZ;
 }
 
 void robotics_math::Eul2Quat(double rz, double ry, double rx, double(&quat)[4])
 {
	 double angles[3] = { rz, ry, rx };
	 double cang[3], sang[3];
	 for (int i = 0; i < 3; i++)
	 {
		 cang[i] = cos(angles[i] / 2.0);
		 sang[i] = sin(angles[i] / 2.0);
	 }
 
	 quat[0] = cang[0] * cang[1] * cang[2] + sang[0] * sang[1] * sang[2];
	 quat[1] = cang[0] * cang[1] * sang[2] - sang[0] * sang[1] * cang[2];
	 quat[2] = cang[0] * sang[1] * cang[2] + sang[0] * cang[1] * sang[2];
	 quat[3] = sang[0] * cang[1] * cang[2] - cang[0] * sang[1] * sang[2];
 }
 
 //void robotics_math::Eul2Quat(const double* eula, Quaternion& quat)
 //{
 //	double rx = eula[0];
 //	double ry = eula[1];
 //	double rz = eula[2];
 //	double angles[3] = { rz, ry, rx };
 //	double cang[3], sang[3];
 //	for (int i = 0; i < 3; i++)
 //	{
 //		cang[i] = cos(angles[i] / 2.0);
 //		sang[i] = sin(angles[i] / 2.0);
 //	}
 //
 //	quat.s_ = cang[0] * cang[1] * cang[2] + sang[0] * sang[1] * sang[2];
 //	quat.x_ = cang[0] * cang[1] * sang[2] - sang[0] * sang[1] * cang[2];
 //	quat.y_ = cang[0] * sang[1] * cang[2] + sang[0] * cang[1] * sang[2];
 //	quat.z_ = sang[0] * cang[1] * cang[2] - cang[0] * sang[1] * sang[2];
 //}
 void robotics_math::Rot2Quat(const matrix::Matrix<double, 3, 3>& mat, double(&quat)[4])
 {
 
	 // RX RY RZ
	 double r11, r12, r13, r21, r22, r23, r31, r32, r33, RX, RY, RZ;
	 r11 = mat(0, 0);
	 r12 = mat(0, 1);
	 r13 = mat(0, 2);
	 r21 = mat(1, 0);
	 r22 = mat(1, 1);
	 r23 = mat(1, 2);
	 r31 = mat(2, 0);
	 r32 = mat(2, 1);
	 r33 = mat(2, 2);
	 if (r11 != 0)
	 {
		 RY = atan2(-r31, sqrt(r11 * r11 + r21 * r21));
		 RX = atan2(r32, r33);
		 RZ = atan2(r21, r11);
	 }
	 else if (r31 < -0)
	 {
		 RY = PI / 2;
		 RX = atan2(r12, r22);
		 RZ = 0;
	 }
	 else
	 {
		 RY = -PI / 2;
		 RX = atan2(-r12, r22);
		 RZ = 0;
	 }
 
	 double angles[3] = { RZ, RY, RX };
	 double cang[3], sang[3];
	 for (int i = 0; i < 3; i++)
	 {
		 cang[i] = cos(angles[i] / 2.0);
		 sang[i] = sin(angles[i] / 2.0);
	 }
 
	 quat[0] = cang[0] * cang[1] * cang[2] + sang[0] * sang[1] * sang[2];
	 quat[1] = cang[0] * cang[1] * sang[2] - sang[0] * sang[1] * cang[2];
	 quat[2] = cang[0] * sang[1] * cang[2] + sang[0] * cang[1] * sang[2];
	 quat[3] = sang[0] * cang[1] * cang[2] - cang[0] * sang[1] * sang[2];
 }
 
 void robotics_math::Quat2Eul(const double(&q)[4], double& rz, double& ry, double& rx)
 {
	 double qw, qx, qy, qz;
	 qw = q[0];
	 qx = q[1];
	 qy = q[2];
	 qz = q[3];
 
	 double siny = -2.0 * (qx * qz - qy * qw);
	 if (siny >= 1.0 - ZERO_VALUE)
	 {
		 rx = 0;
		 ry = PI / 2.0;
		 rz = -2.0 * atan2(qx, qw);
	 }
	 else if (siny <= -1.0 + ZERO_VALUE)
	 {
		 rx = 0;
		 ry = -PI / 2.0;
		 rz = 2.0 * atan2(qx, qw);
	 }
	 else
	 {
		 rx = atan2(2.0 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);
		 ry = asin(siny);
		 rz = atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
	 }
 }
 
 //void robotics_math::Quat2Eul(const Quaternion& q, double* eula)
 //{
 //	double qw = q.s_;
 //	double qx = q.x_;
 //	double qy = q.y_;
 //	double qz = q.z_;
 //	double& rx = eula[0];
 //	double& ry = eula[1];
 //	double& rz = eula[2];
 //
 //	double siny = -2.0 * (qx * qz - qy * qw);
 //	if (siny >= 1.0 - 1e-10)
 //	{
 //		rx = 0;
 //		ry = PI / 2.0;
 //		rz = -2.0 * atan2(qx, qw);
 //	}
 //	else if (siny <= -1.0 + 1e-10)
 //	{
 //		rx = 0;
 //		ry = -PI / 2.0;
 //		rz = 2.0 * atan2(qx, qw);
 //	}
 //	else
 //	{
 //		rx = atan2(2.0 * (qy * qz + qw * qx), qw * qw - qx * qx - qy * qy + qz * qz);
 //		ry = asin(siny);
 //		rz = atan2(2.0 * (qx * qy + qw * qz), qw * qw + qx * qx - qy * qy - qz * qz);
 //	}
 //}
 
 void robotics_math::Quat2Mat(const double(&q)[4], matrix::Matrix<double, 4, 4>& mat)
 {
	 double qw, qx, qy, qz;
	 qw = q[0];
	 qx = q[1];
	 qy = q[2];
	 qz = q[3];
 
	 mat(0, 0) = 1.0 - 2.0 * qy * qy - 2.0 * qz * qz;
	 mat(0, 1) = 2.0 * (qx * qy - qz * qw);
	 mat(0, 2) = 2.0 * (qx * qz + qy * qw);
	 mat(1, 0) = 2.0 * (qx * qy + qz * qw);
	 mat(1, 1) = 1.0 - 2.0 * qx * qx - 2.0 * qz * qz;
	 mat(1, 2) = 2.0 * (qy * qz - qx * qw);
	 mat(2, 0) = 2.0 * (qx * qz - qy * qw);
	 mat(2, 1) = 2.0 * (qy * qz + qx * qw);
	 mat(2, 2) = 1.0 - 2.0 * qx * qx - 2.0 * qy * qy;
 
	 mat(3, 3) = 1.0;
 }
 
 void robotics_math::Quat2Rot(const double(&q)[4], matrix::Matrix<double, 3, 3>& mat)
 {
	 double qw, qx, qy, qz;
	 qw = q[0];
	 qx = q[1];
	 qy = q[2];
	 qz = q[3];
 
	 mat(0, 0) = 1.0 - 2.0 * qy * qy - 2.0 * qz * qz;
	 mat(0, 1) = 2.0 * (qx * qy - qz * qw);
	 mat(0, 2) = 2.0 * (qx * qz + qy * qw);
	 mat(1, 0) = 2.0 * (qx * qy + qz * qw);
	 mat(1, 1) = 1.0 - 2.0 * qx * qx - 2.0 * qz * qz;
	 mat(1, 2) = 2.0 * (qy * qz - qx * qw);
	 mat(2, 0) = 2.0 * (qx * qz - qy * qw);
	 mat(2, 1) = 2.0 * (qy * qz + qx * qw);
	 mat(2, 2) = 1.0 - 2.0 * qx * qx - 2.0 * qy * qy;
 }
 
 void robotics_math::slerp(const double(&q1)[4], const double(&q2)[4], const double& rot_planing, double(&quat)[4])
 {
	 matrix::Vector<double, 4> q1_vec(q1);
	 matrix::Vector<double, 4> q2_vec(q2);
	 matrix::Vector<double, 4> quat_vec;
	 double dot_product;
	 dot_product = q1_vec * q2_vec;
	 if (std::abs(dot_product) > 0.9995)
	 {
		 quat_vec = (1 - rot_planing) * q1_vec + rot_planing * q2_vec;
		 for (int i = 0; i < 4; i++)
		 {
			 quat[i] = quat_vec(i);
		 }
		 return;
	 }
	 double theta_zero;
	 theta_zero = std::acos(dot_product);
	 double theta;
	 theta = theta_zero * rot_planing;
	 double s_zero;
	 s_zero = cos(theta) - dot_product * sin(theta) / sin(theta_zero);
	 double s_one;
	 s_one = sin(theta) / sin(theta_zero);
	 quat_vec = s_zero * q1_vec + s_one * q2_vec;
	 for (int i = 0; i < 4; i++)
	 {
		 quat[i] = quat_vec(i);
	 }
 }
 
 void robotics_math::slerpInsert(const double(&p)[4], const double(&q)[4], const double& real_dis, double(&quat)[4])
 {
	 double w0 = p[0];
	 double x0 = p[1];
	 double y0 = p[2];
	 double z0 = p[3];
 
	 double w1 = q[0];
	 double x1 = q[1];
	 double y1 = q[2];
	 double z1 = q[3];
 
	 double cosOmega, sinOmega, omega, oneOverSinOmega;
	 double k0, k1;
 
	 cosOmega = w0 * w1 + x0 * x1 + y0 * y1 + z0 * z1;
 
	 if (cosOmega < 0.0)
	 {
		 w1 = -w1;
		 x1 = -x1;
		 y1 = -y1;
		 z1 = -z1;
		 cosOmega = -cosOmega;
	 }
 
	 if (cosOmega - 1 > -ZERO_VALUE)
	 {
		 k0 = 1.0 - real_dis;
		 k1 = real_dis;
	 }
	 else
	 {
		 sinOmega = sqrt(1.0 - cosOmega * cosOmega);
		 omega = atan2(sinOmega, cosOmega);
		 oneOverSinOmega = 1.0 / sinOmega;
		 k0 = sin((1.0 - real_dis) * omega) * oneOverSinOmega;
		 k1 = sin(real_dis * omega) * oneOverSinOmega;
	 }
 
	 quat[0] = w0 * k0 + w1 * k1;
	 quat[1] = x0 * k0 + x1 * k1;
	 quat[2] = y0 * k0 + y1 * k1;
	 quat[3] = z0 * k0 + z1 * k1;
 }
 
 matrix::SquareMatrix<double, 3> robotics_math::frameToRotation(const matrix::SquareMatrix<double, 4>& frame_in)
 {
	 matrix::SquareMatrix<double, 3> rotation_out;
	 for (int i = 0; i < 3; i++)
	 {
		 for (int j = 0; j < 3; j++)
		 {
			 rotation_out(i, j) = frame_in(i, j);
		 }
	 }
	 return rotation_out;
 }
 
 matrix::Vector<double, 3> robotics_math::frameToPosition(const matrix::SquareMatrix<double, 4>& frame_in)
 {
	 matrix::Vector<double, 3> position_out;
	 for (int i = 0; i < 3; i++)
	 {
		 position_out(i) = frame_in(i, 3);
	 }
	 return position_out;
 }
 
 matrix::SquareMatrix<double, 4> robotics_math::rotAndPosToFrame(const matrix::SquareMatrix<double, 3>& rot_in, const matrix::Vector<double, 3>& pos_in)
 {
	 matrix::SquareMatrix<double, 4> frame_out;
	 for (int i = 0; i < 3; i++)
	 {
		 for (int j = 0; j < 3; j++)
		 {
			 frame_out(i, j) = rot_in(i, j);
		 }
	 }
	 for (int i = 0; i < 3; i++)
	 {
 
		 frame_out(i, 3) = pos_in(i);
	 }
	 return frame_out;
 }
 
 matrix::Vector<double, 3> robotics_math::cross(const matrix::Vector<double, 3>& A, const matrix::Vector<double, 3>& B)
 {
	 matrix::Vector<double, 3> C;
	 C(0) = A(1) * B(2) - A(2) * B(1);
	 C(1) = A(2) * B(0) - A(0) * B(2);
	 C(2) = A(0) * B(1) - A(1) * B(0);
	 return C;
 }
 
 void robotics_math::arrayCross(const double* p1, const double* p2, double* result)
 {
	 double i = p1[1] * p2[2] - p2[1] * p1[2];
	 double j = -(p1[0] * p2[2] - p2[0] * p1[2]);
	 double k = p1[0] * p2[1] - p2[0] * p1[1];
	 double temp[3] = { i,j,k };
	 for (int i = 0; i < 3; i++)
		 result[i] = temp[i];
 }
 
 void robotics_math::InvQuat(const double(&q)[4], double(&InvQuat)[4])
 {
	 double norm = sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	 InvQuat[0] = q[0] / norm;
	 InvQuat[1] = -q[1] / norm;
	 InvQuat[2] = -q[2] / norm;
	 InvQuat[3] = -q[3] / norm;
 }
 
 void robotics_math::GetQuaAxis(const double(&quat)[4], double(&QuaAxis)[4])
 {
	 double vec[3] = { 0, 0, 0 };
	 double theta = 0.0, temp = 0.0;
	 if (quat[0] > 1)
	 {
		 theta = acos(1);
	 }
	 else if (quat[0] < -1)
	 {
		 theta = acos(-1);
	 }
	 else
	 {
		 theta = acos(quat[0]);
	 }
 
	 if (fabs(theta) <= 1e-10)
	 {
		 vec[0] = 0.0;
		 vec[1] = 0.0;
		 vec[2] = 0.0;
	 }
	 else
	 {
		 temp = 1.0 / sin(theta);
		 vec[0] = quat[1] * temp;
		 vec[1] = quat[2] * temp;
		 vec[2] = quat[3] * temp;
	 }
 
	 if (2.0 * theta < PI)
	 {
		 QuaAxis[0] = 2.0 * theta;
		 QuaAxis[1] = vec[0];
		 QuaAxis[2] = vec[1];
		 QuaAxis[3] = vec[2];
	 }
	 else
	 {
		 QuaAxis[0] = 2.0 * (PI - theta);
		 QuaAxis[1] = -vec[0];
		 QuaAxis[2] = -vec[1];
		 QuaAxis[3] = -vec[2];
	 }
 }
 
 void robotics_math::MultiQuat(const double(&q)[4], const double(&r)[4], double(&result)[4])
 {
	 //% vec = s1 * v2 + s2 * v1 + cross(v1, v2)
	 //	vec = [q(1)*r(2) q(1)*r(3) q(1)*r(4)] + ...
	 //	[r(1)*q(2) r(1)*q(3) r(1)*q(4)] + ...
	 //	[q(3)*r(4) - q(4)*r(3) ...
	 //	q(4)*r(2) - q(2)*r(4) ...
	 //	q(2)*r(3) - q(3)*r(2)];
 
	 //% Calculate scalar portion of quaternion product
	 //	% scalar = s1 * s2 - dot(v1, v2)
	 //	scalar = q(1)*r(1) - q(2)*r(2) - q(3)*r(3) - q(4)*r(4);
 
	 //qout = [scalar  vec];
	 double vec[3] = { 0, 0, 0 };
	 vec[0] = q[0] * r[1] + r[0] * q[1] + q[2] * r[3] - q[3] * r[2];
	 vec[1] = q[0] * r[2] + r[0] * q[2] + q[3] * r[1] - q[1] * r[3];
	 vec[2] = q[0] * r[3] + r[0] * q[3] + q[1] * r[2] - q[2] * r[1];
 
	 double scalar = q[0] * r[0] - q[1] * r[1] - q[2] * r[2] - q[3] * r[3];
 
	 result[0] = scalar;
	 result[1] = vec[0];
	 result[2] = vec[1];
	 result[3] = vec[2];
 }
 

 void robotics_math::mat4Inv(const matrix::Matrix<double, 4, 4>& src_T, matrix::Matrix<double, 4, 4>& des_T)
 {
	 des_T.identity();
	 for (int i = 0; i < 3; i++)
	 {
		 for (int j = 0; j < 3; j++)
		 {
			 des_T(i, j) = src_T(j, i);
		 }
	 }
	 des_T(0, 3) = -(src_T(0, 0) * src_T(0, 3) + src_T(1, 0) * src_T(1, 3) + src_T(2, 0) * src_T(2, 3));
	 des_T(1, 3) = -(src_T(0, 1) * src_T(0, 3) + src_T(1, 1) * src_T(1, 3) + src_T(2, 1) * src_T(2, 3));
	 des_T(2, 3) = -(src_T(0, 2) * src_T(0, 3) + src_T(1, 2) * src_T(1, 3) + src_T(2, 2) * src_T(2, 3));
 }
