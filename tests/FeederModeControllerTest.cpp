#include "FeederModeController.hpp"

#include <cassert>

int main()
{
    FeederModeController controller(-120.0f);

    // 普通状态保持位置控制，单发触发仍由原位置控制器处理。
    FeederModeOutput output = controller.update(3, 1, 660);
    assert(output.mode == FeederControlMode::Position);
    assert(output.continuous_speed_target == 0.0f);
    assert(!output.entered_continuous);
    assert(!output.left_continuous);

    // S2=2且左摇杆推过阈值时，进入-120 rpm连发速度模式。
    output = controller.update(3, 2, 660);
    assert(output.mode == FeederControlMode::ContinuousSpeed);
    assert(output.continuous_speed_target == -120.0f);
    assert(output.entered_continuous);
    assert(!output.left_continuous);

    // 持续连发不应重复触发模式切换。
    output = controller.update(3, 2, 660);
    assert(output.mode == FeederControlMode::ContinuousSpeed);
    assert(!output.entered_continuous);
    assert(!output.left_continuous);

    // 松开摇杆后回到位置模式，并通知任务同步当前位置。
    output = controller.update(3, 2, 0);
    assert(output.mode == FeederControlMode::Position);
    assert(output.left_continuous);

    // S1=1的Stop优先级最高；从连发进入Stop也必须报告退出连发。
    controller.update(3, 2, 660);
    output = controller.update(1, 2, 660);
    assert(output.mode == FeederControlMode::Stop);
    assert(output.left_continuous);
    assert(output.continuous_speed_target == 0.0f);

    return 0;
}
