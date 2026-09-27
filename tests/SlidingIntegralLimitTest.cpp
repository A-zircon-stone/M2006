#include "Sliding.h"

#include <cassert>

int main()
{
    cSMC controller;
    controller.Init();
    controller.Integval_Clear();

    // 持续正位置误差不能让积分超过预设上限。
    for (int i = 0; i < 2000; ++i)
    {
        controller.ErrorUpdate(0.0f, 1000.0f, 0.0f);
    }
    assert(controller.getSmc().error.p_error_integral ==
           P_EORROR_INTEGRAL_MAX);

    controller.Integval_Clear();

    // 持续负位置误差也必须被限制在对称下限。
    for (int i = 0; i < 2000; ++i)
    {
        controller.ErrorUpdate(0.0f, -1000.0f, 0.0f);
    }
    assert(controller.getSmc().error.p_error_integral ==
           -P_EORROR_INTEGRAL_MAX);

    // EISMC 输出已经负向饱和，而正位置误差还会把输出继续推向负饱和时，
    // 必须暂停积分，避免电机停住后 surface 仍不断增大。
    controller.Init();
    controller.Integval_Clear();
    controller.SetParam(0.3f,
                        2000.0f,
                        2.0f,
                        10.0f,
                        1.1f,
                        0.5f,
                        1500.0f,
                        EISMC,
                        0.0f);
    controller.ErrorUpdate(0.0f, 1000.0f, 0.0f);
    controller.SmcCalculate();
    assert(controller.Out() == -1500.0f);
    const float positive_integral =
            controller.getSmc().error.p_error_integral;
    controller.ErrorUpdate(0.0f, 1000.0f, 0.0f);
    assert(controller.getSmc().error.p_error_integral == positive_integral);

    // 正向饱和时，对称地禁止负误差继续把积分推向正向饱和。
    controller.Integval_Clear();
    controller.ErrorUpdate(0.0f, -1000.0f, 0.0f);
    controller.SmcCalculate();
    assert(controller.Out() == 1500.0f);
    const float negative_integral =
            controller.getSmc().error.p_error_integral;
    controller.ErrorUpdate(0.0f, -1000.0f, 0.0f);
    assert(controller.getSmc().error.p_error_integral == negative_integral);

    return 0;
}
