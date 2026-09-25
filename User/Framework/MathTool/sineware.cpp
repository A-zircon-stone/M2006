//
// Created by hx on 2025/5/19.
//

#include "sineware.hpp"
// 更新角度并返回正弦值
double SineWaveGenerator::updateAndGetSineValue()
{
    angle += angularFrequency * 0.005; // 根据时间间隔更新角度
    // 计算并返回正弦值，通过振幅和偏移量调整
    return amplitude * sin(angle) + offset;
}

// 设置正弦波的周期
void SineWaveGenerator::setPeriod(double period)
{
    angularFrequency = 2.0 * M_PI / period;
}

void SineWaveGenerator::clear()
{
    if (angle == 0.0f)
        return;

    angle = 0.0;
}