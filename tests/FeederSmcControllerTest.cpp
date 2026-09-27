#include "FeederSmcController.hpp"

#include <cassert>
#include <cmath>

int main()
{
    FeederSmcController controller(0.3f,
                                   2000.0f,
                                   2.0f,
                                   10.0f,
                                   1.1f,
                                   0.5f,
                                   9600.0f,
                                   2000.0f);

    // 12°目标在2ms内变化，对应1000rpm；防止把°/s直接当成rpm。
    controller.reset(0.0f);
    const float step_output = controller.update(12.0f, 0.0f, 0.0f, 0.002f);
    assert(std::fabs(controller.targetSpeedRpm() - 1000.0f) < 0.001f);
    assert(step_output == 9600.0f);

    // 本周期预计输出已经饱和时，本次位置积分不得被接受。
    controller.reset(0.0f);
    const float saturated_output =
            controller.update(0.0f, 1000.0f, 0.0f, 0.002f);
    assert(saturated_output == -9600.0f);
    assert(controller.positionErrorIntegral() == 0.0f);

    // 未饱和的小误差仍应正常积分。
    controller.reset(0.0f);
    controller.update(0.0f, 0.00001f, 0.0f, 0.002f);
    assert(std::fabs(controller.positionErrorIntegral() - 0.00000002f) <
           0.000000001f);

    // 位置接近目标但仍有正速度时，必须输出负电流制动，不能因位置死区归零。
    controller.reset(0.0f);
    const float braking_output =
            controller.update(0.0f, 0.1f, 100.0f, 0.002f);
    assert(braking_output < 0.0f);

    return 0;
}
