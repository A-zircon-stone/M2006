#include "FeederFireController.hpp"

#include <cassert>

int main()
{
    FeederFireController controller(-36.0f, 1.0f, 30);
    controller.reset(100.0f, false);

    // S1=1 的 Stop 状态把目标同步到当前位置，但不得进入发射步骤。
    FeederFireOutput output = controller.update(1, 1, 660, 100.0f, true);
    assert(output.stop);
    assert(output.target_changed);
    assert(!output.step_in_progress);
    assert(output.target_position == 100.0f);

    // 单发模式只响应一次上升沿；摇杆持续保持不能创建第二发。
    controller.update(3, 1, 0, 100.0f, true);
    output = controller.update(3, 1, 660, 100.0f, true);
    assert(!output.stop);
    assert(output.target_changed);
    assert(output.step_in_progress);
    assert(output.target_position == 64.0f);

    output = controller.update(3, 1, 660, 64.0f, true);
    assert(!output.target_changed);
    for (int i = 0; i < 31; ++i)
    {
        output = controller.update(3, 1, 660, 64.0f, true);
        assert(!output.target_changed);
    }

    // 松开后重新推到触发区，才允许第二次单发。
    controller.update(3, 1, 0, 64.0f, true);
    output = controller.update(3, 1, 660, 64.0f, true);
    assert(output.target_changed);
    assert(output.target_position == 28.0f);

    // 连发模式保持触发时，必须等上一发到位并完成30周期间隔。
    controller.reset(100.0f, false);
    output = controller.update(3, 2, 660, 100.0f, true);
    assert(output.target_changed);
    assert(output.target_position == 64.0f);

    output = controller.update(3, 2, 660, 80.0f, true);
    assert(!output.target_changed);
    assert(output.step_in_progress);

    // 实际位置可能越过理论目标；原版下一发以实际位置为基准。
    output = controller.update(3, 2, 660, 59.0f, true);
    assert(!output.target_changed);
    assert(!output.step_in_progress);

    for (int i = 0; i < 29; ++i)
    {
        output = controller.update(3, 2, 660, 59.0f, true);
        assert(!output.target_changed);
    }
    output = controller.update(3, 2, 660, 59.0f, true);
    assert(output.target_changed);
    assert(output.target_position == 23.0f);

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

    // 新拨弹盘反向安装：正步长时应向角度增大的方向运动，并按正向到位。
    FeederFireController reversed_controller(40.0f, 1.0f, 30);
    reversed_controller.reset(100.0f, false);
    output = reversed_controller.update(3, 2, 660, 100.0f, true);
    assert(output.target_changed);
    assert(output.target_position == 140.0f);

    output = reversed_controller.update(3, 2, 660, 120.0f, true);
    assert(output.step_in_progress);

    output = reversed_controller.update(3, 2, 660, 140.0f, true);
    assert(!output.step_in_progress);

    return 0;
}
