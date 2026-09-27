#include "FeederPidSafety.hpp"

#include <cassert>

int main()
{
    // 新拨弹盘只允许负电流，正电流与零电流都输出 0。
    assert(FeederPid_LimitForwardCurrent(200.0f, 1500.0f) == 0);
    assert(FeederPid_LimitForwardCurrent(0.0f, 1500.0f) == 0);
    assert(FeederPid_LimitForwardCurrent(-725.0f, 1500.0f) == -725);
    // 负电流下限为 -1500。
    assert(FeederPid_LimitForwardCurrent(-2000.0f, 1500.0f) == -1500);

    // 按原版控制链，SMC 的正负输出都允许发送，只做对称限幅。
    assert(FeederSmc_LimitBidirectionalCurrent(-725.0f, 1500.0f) == -725);
    assert(FeederSmc_LimitBidirectionalCurrent(-2000.0f, 1500.0f) == -1500);
    assert(FeederSmc_LimitBidirectionalCurrent(300.0f, 1500.0f) == 300);
    assert(FeederSmc_LimitBidirectionalCurrent(2000.0f, 1500.0f) == 1500);
    return 0;
}
