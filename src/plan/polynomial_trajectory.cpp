#include "polynomial_trajectory.h"

// 计算五次多项式的系数
std::vector<double> polynomial_trajectory::calculateCoefficients(double p0, double v0, double a0, double pf, double vf, double af, double T)
{
    std::vector<double> A(6, 0.0);
    A[0] = p0;
    A[1] = v0;
    A[2] = a0 / 2.0;

    double B[6][6] = {
        {1, T, T * T, T * T * T, T * T * T * T, T * T * T * T * T},
        {0, 1, 2 * T, 3 * T * T, 4 * T * T * T, 5 * T * T * T * T},
        {0, 0, 2, 6 * T, 12 * T * T, 20 * T * T * T},
        {1, 0, 0, 0, 0, 0},
        {0, 1, 0, 0, 0, 0},
        {0, 0, 2, 0, 0, 0}};

    double b[6] = {pf, vf, af, p0, v0, a0};

    // 使用高斯消元法求解线性方程组
    for (int i = 0; i < 6; ++i)
    {
        double max_val = B[i][i];
        int max_row = i;
        for (int j = i + 1; j < 6; ++j)
        {
            if (fabs(B[j][i]) > fabs(max_val))
            {
                max_val = B[j][i];
                max_row = j;
            }
        }

        if (max_row != i)
        {
            for (int k = 0; k < 6; ++k)
            {
                std::swap(B[i][k], B[max_row][k]);
            }
            std::swap(b[i], b[max_row]);
        }

        for (int j = i + 1; j < 6; ++j)
        {
            double factor = B[j][i] / B[i][i];
            for (int k = i; k < 6; ++k)
            {
                B[j][k] -= factor * B[i][k];
            }
            b[j] -= factor * b[i];
        }
    }

    for (int i = 5; i >= 0; --i)
    {
        for (int j = i + 1; j < 6; ++j)
        {
            b[i] -= B[i][j] * A[j];
        }
        A[i] = b[i] / B[i][i];
    }

    return A;
}

// 计算多项式在t时刻的位置、速度和加速度
std::tuple<double, double, double> polynomial_trajectory::evaluatePolynomial(const std::vector<double> &A, double t)
{
    double p = 0.0, v = 0.0, a = 0.0;
    for (int i = 0; i < 6; ++i)
    {
        p += A[i] * pow(t, i);
        v += A[i] * i * pow(t, i - 1);
        a += A[i] * i * (i - 1) * pow(t, i - 2);
    }
    return std::make_tuple(p, v, a);
}