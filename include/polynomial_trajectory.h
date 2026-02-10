#ifndef _POLYNIMIAL_TRAJECTORT_H_
#define _POLYNIMIAL_TRAJECTORT_H_

#include <vector>
#include <cassert>
#include <iostream>
#include <fstream>
#include <vector>
#include <sstream>
#include <tuple>
#include <cmath>
namespace polynomial_trajectory
{

// 计算五次多项式的系数
std::vector<double> calculateCoefficients(double p0, double v0, double a0, double pf, double vf, double af, double T);
// 计算多项式在t时刻的位置、速度和加速度
std::tuple<double, double, double> evaluatePolynomial(const std::vector<double> &A, double t);

}

#endif