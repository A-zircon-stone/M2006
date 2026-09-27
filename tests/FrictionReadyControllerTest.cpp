#include "FrictionReadyController.hpp"

#include <cassert>

int main()
{
    FrictionReadyController controller(5900.0f, 6300.0f, 20);

    // 未使能摩擦轮时，即使反馈转速碰巧在范围内，也不能允许发射。
    for (int i = 0; i < 20; ++i)
    {
        assert(!controller.update(false, -6200.0f, 6200.0f));
    }

    // 必须左右轮同时连续达标20个控制周期。
    for (int i = 0; i < 19; ++i)
    {
        assert(!controller.update(true, -6200.0f, 6200.0f));
    }
    assert(controller.update(true, -6200.0f, 6200.0f));
    assert(controller.ready());

    // 任意一侧掉出范围，许可必须立即撤销。
    assert(!controller.update(true, -5800.0f, 6200.0f));
    assert(!controller.ready());

    // 掉速后必须重新连续达标20个周期，不能沿用旧计数。
    for (int i = 0; i < 19; ++i)
    {
        assert(!controller.update(true, -6200.0f, 6200.0f));
    }
    assert(controller.update(true, -6200.0f, 6200.0f));

    // 关闭摩擦轮会清空许可和累计计数。
    assert(!controller.update(false, -6200.0f, 6200.0f));
    assert(!controller.ready());

    return 0;
}
