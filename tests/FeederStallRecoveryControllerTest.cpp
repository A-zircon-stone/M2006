#include "FeederStallRecoveryController.hpp"

#include <cassert>

int main()
{
    FeederStallRecoveryController controller(1400,
                                              5.0f,
                                              100,
                                              36.0f,
                                              400);

    // 满电流但电机仍在正常运动时，不应误判堵转。
    FeederStallRecoveryOutput output{};
    for (int i = 0; i < 120; ++i)
    {
        output = controller.update(-1500, -20.0f, -100.0f);
        assert(!output.stalled);
    }

    // 满电流且低速持续100个周期后，进入反拨并以实际位置+36°为目标。
    for (int i = 0; i < 99; ++i)
    {
        output = controller.update(-1500, 0.0f, -100.0f);
        assert(!output.stalled);
    }
    output = controller.update(-1500, 0.0f, -100.0f);
    assert(output.stalled);
    assert(output.clear_integral);
    assert(output.target_position == -64.0f);

    // 反拨期间保持同一目标，并要求每周期清除原发射方向的积分。
    output = controller.update(1500, 10.0f, -80.0f);
    assert(output.stalled);
    assert(output.clear_integral);
    assert(output.target_position == -64.0f);

    // 到达反拨目标后退出解卡，本周期不应立即继续发射。
    output = controller.update(1500, 0.0f, -64.0f);
    assert(!output.stalled);
    assert(output.recovery_completed);
    assert(output.clear_integral);

    // 反拨卡住时，最多保持400个周期，随后超时退出。
    controller.reset();
    for (int i = 0; i < 100; ++i)
    {
        output = controller.update(-1500, 0.0f, 50.0f);
    }
    assert(output.stalled);
    for (int i = 0; i < 399; ++i)
    {
        output = controller.update(1500, 0.0f, 50.0f);
        assert(output.stalled);
    }
    output = controller.update(1500, 0.0f, 50.0f);
    assert(!output.stalled);
    assert(output.recovery_completed);

    // 正常拨弹改为正方向后，反拨步长为-36°，到位判断也必须反向。
    FeederStallRecoveryController reversed_controller(1400,
                                                       5.0f,
                                                       100,
                                                       -36.0f,
                                                       400);
    for (int i = 0; i < 100; ++i)
    {
        output = reversed_controller.update(1500, 0.0f, 100.0f);
    }
    assert(output.stalled);
    assert(output.target_position == 64.0f);

    output = reversed_controller.update(-1500, -10.0f, 80.0f);
    assert(output.stalled);

    output = reversed_controller.update(-1500, 0.0f, 64.0f);
    assert(!output.stalled);
    assert(output.recovery_completed);

    return 0;
}
