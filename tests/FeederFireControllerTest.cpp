#include "FeederFireController.hpp"

#include <cassert>

int main()
{
    FeederFireController controller(-30.0f, 1.0f, 30);
    controller.reset(100.0f, false);

    // S1=1 的 Stop 状态把目标同步到当前位置，但不得进入发射步骤。
    FeederFireOutput output = controller.update(1, 1, 660, 100.0f);
    assert(output.stop);
    assert(output.target_changed);
    assert(!output.step_in_progress);
    assert(output.target_position == 100.0f);

    // 单发模式只响应一次上升沿；摇杆持续保持不能创建第二发。
    controller.update(3, 1, 0, 100.0f);
    output = controller.update(3, 1, 660, 100.0f);
    assert(!output.stop);
    assert(output.target_changed);
    assert(output.step_in_progress);
    assert(output.target_position == 70.0f);

    output = controller.update(3, 1, 660, 70.0f);
    assert(!output.target_changed);
    for (int i = 0; i < 31; ++i)
    {
        output = controller.update(3, 1, 660, 70.0f);
        assert(!output.target_changed);
    }

    // 松开后重新推到触发区，才允许第二次单发。
    controller.update(3, 1, 0, 70.0f);
    output = controller.update(3, 1, 660, 70.0f);
    assert(output.target_changed);
    assert(output.target_position == 40.0f);

    // 连发模式保持触发时，必须等上一发到位并完成30周期间隔。
    controller.reset(100.0f, false);
    output = controller.update(3, 2, 660, 100.0f);
    assert(output.target_changed);
    assert(output.target_position == 70.0f);

    output = controller.update(3, 2, 660, 80.0f);
    assert(!output.target_changed);
    assert(output.step_in_progress);

    output = controller.update(3, 2, 660, 70.0f);
    assert(!output.target_changed);
    assert(!output.step_in_progress);

    for (int i = 0; i < 29; ++i)
    {
        output = controller.update(3, 2, 660, 70.0f);
        assert(!output.target_changed);
    }
    output = controller.update(3, 2, 660, 70.0f);
    assert(output.target_changed);
    assert(output.target_position == 40.0f);

    return 0;
}
