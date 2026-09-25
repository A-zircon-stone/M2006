//
// Created by hx on 2025/5/19.
//

#ifndef FZY_ONMI_GIMBAL_SINEWARE_HPP
#define FZY_ONMI_GIMBAL_SINEWARE_HPP

#include <cmath>

class SineWaveGenerator {
private:
    double angle; // 当前角度
    double angularFrequency; // 角频率（每秒钟转过的弧度数）
    double amplitude; // 正弦波的振幅
    double offset; // 正弦波的偏移量

public:
    SineWaveGenerator(double period, double amplitude = 180.0, double offset = 0.0)
            : angle(0.0)
            , angularFrequency(2.0 * M_PI / period)
            , amplitude(amplitude)
            , offset(offset)
    {
    }

    // 更新角度并返回正弦值
    double updateAndGetSineValue();

    // 设置正弦波的周期
    void setPeriod(double period);

    void clear();
};
class sineware {

};


#endif //FZY_ONMI_GIMBAL_SINEWARE_HPP
