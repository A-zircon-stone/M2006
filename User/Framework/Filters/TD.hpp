//
// Created by Lenovo on 2024/4/28.
//

#ifndef KOSANN_UAVGIMBAL_TD_HPP
#define KOSANN_UAVGIMBAL_TD_HPP
#include "filter.hpp"


class TD
{
private:
    // TD微分跟踪器参数
    const double dt = 0.01; // 采样时间间隔
    const double r = 40;   // 跟踪因子
    const double h = 0.01;  // 滤波因子

    // TD微分跟踪器状态变量
    double x1 = 0.0; // 输入信号的估计值
    double x2 = 0.0; // 输入信号的导数估计值
private:
    // TD微分跟踪器的更新函数
    void UpdateTD(double u) {
        double d = r * h;
        double d0 = d * h;
        double y = x1 - u + h * x2;
        double a0 = sqrt(d * d + 8 * r * std::abs(y));

        double a;
        if (std::abs(y) <= d0) {
            a = x2 + y / h;
        } else {
            a = x2 + 0.5 * (a0 - d) * (y / std::abs(y));
        }

        double f = -r * (a / d);
        x1 = x1 + dt * x2;
        x2 = x2 + dt * f;
    }
public:
    double Return_TD_x(double u)
    {
        UpdateTD(u);
        return x1;
    };
    double Return_TD_de(double u)
    {
        UpdateTD(u);
        return x2;
    }
};

#endif //KOSANN_UAVGIMBAL_TD_HPP
