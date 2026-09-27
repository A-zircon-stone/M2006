#include "FeederFireController.hpp"

#include <cassert>

int main()
{
    FeederFireController controller(45.0f, 0.5f, 70, 10);
    controller.reset(100.0f, false);

    // S1=1 的 Stop 状态把目标同步到当前位置，但不得进入发射步骤。
    FeederFireOutput output = controller.update(1, 1, 660, 100.0f, true);
    assert(output.stop);
    assert(output.target_changed);
    assert(!output.step_in_progress);
    assert(output.target_position == 100.0f);
    assert(!output.shot_completed);

    // 单发模式只响应一次上升沿，目标必须从实际位置增加45°。
    controller.update(3, 1, 0, 100.0f, true);
    output = controller.update(3, 1, 660, 100.0f, true);
    assert(!output.stop);
    assert(output.target_changed);
    assert(output.step_in_progress);
    assert(output.target_position == 145.0f);

    // 即使提前到位，也必须至少维持70个控制周期。
    for (int i = 0; i < 69; ++i)
    {
        output = controller.update(3, 1, 660, 145.0f, true);
        assert(!output.target_changed);
        assert(output.step_in_progress);
        assert(!output.shot_completed);
    }

    // 第70周期开始到位暂停，但还要继续保持目标10个周期。
    output = controller.update(3, 1, 660, 145.0f, true);
    assert(output.step_in_progress);
    assert(!output.shot_completed);

    for (int i = 0; i < 8; ++i)
    {
        output = controller.update(3, 1, 660, 145.0f, true);
        assert(output.step_in_progress);
        assert(!output.shot_completed);
    }

    // 第10个暂停周期才结束本发，同步实际位置并通知任务清积分。
    output = controller.update(3, 1, 660, 146.0f, true);
    assert(output.target_changed);
    assert(!output.step_in_progress);
    assert(output.shot_completed);
    assert(output.target_position == 146.0f);

    // 单发摇杆一直保持时不得创建第二发。
    output = controller.update(3, 1, 660, 146.0f, true);
    assert(!output.target_changed);

    // 摩擦轮未达标时不允许创建新目标。单发触发不会排队，必须松开后重触发。
    controller.reset(100.0f, false);
    output = controller.update(3, 1, 660, 100.0f, false);
    assert(!output.target_changed);
    assert(!output.step_in_progress);
    output = controller.update(3, 1, 660, 100.0f, true);
    assert(!output.target_changed);
    controller.update(3, 1, 0, 100.0f, true);
    output = controller.update(3, 1, 660, 100.0f, true);
    assert(output.target_changed);

    // 连发保持触发时，掉速期间暂停；恢复许可后自动创建下一发。
    controller.reset(100.0f, false);
    output = controller.update(3, 2, 660, 100.0f, false);
    assert(!output.target_changed);
    output = controller.update(3, 2, 660, 100.0f, true);
    assert(output.target_changed);

    return 0;
}
