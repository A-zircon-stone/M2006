#include "FeederStallRecoveryController.hpp"

#include <cassert>

int main()
{
    FeederStallRecoveryController controller(7500.0f,
                                              100,
                                              -45.0f,
                                              400);

    // 反馈电流没有超过阈值时，无论速度如何都不能判定堵转。
    FeederStallRecoveryOutput output{};
    for (int i = 0; i < 120; ++i)
    {
        output = controller.update(7500.0f, 100.0f);
        assert(!output.stalled);
    }

    // 反馈电流绝对值超过7500并持续100个周期后，反拨45°。
    for (int i = 0; i < 99; ++i)
    {
        output = controller.update(-7600.0f, 100.0f);
        assert(!output.stalled);
    }
    output = controller.update(-7600.0f, 100.0f);
    assert(output.stalled);
    assert(output.clear_integral);
    assert(output.target_position == 55.0f);

    // 反拨期间保持同一目标，并要求每周期清除原发射方向的积分。
    output = controller.update(0.0f, 80.0f);
    assert(output.stalled);
    assert(output.clear_integral);
    assert(output.target_position == 55.0f);

    // 到达反拨目标后退出解卡，本周期不应立即继续发射。
    output = controller.update(0.0f, 55.0f);
    assert(!output.stalled);
    assert(output.recovery_completed);
    assert(output.clear_integral);

    // 反拨卡住时，最多保持400个周期，随后超时退出。
    controller.reset();
    for (int i = 0; i < 100; ++i)
    {
        output = controller.update(7600.0f, 50.0f);
    }
    assert(output.stalled);
    for (int i = 0; i < 399; ++i)
    {
        output = controller.update(0.0f, 50.0f);
        assert(output.stalled);
    }
    output = controller.update(0.0f, 50.0f);
    assert(!output.stalled);
    assert(output.recovery_completed);

    return 0;
}
