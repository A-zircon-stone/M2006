#include "FeederPidSafety.hpp"

#include <cmath>

int16_t FeederPid_LimitForwardCurrent(const float pid_output,
                                      const float max_current)
{
    if (!std::isfinite(pid_output) || !std::isfinite(max_current) ||
        pid_output >= 0.0f || max_current <= 0.0f)
    {
        return 0;
    }

    // 新拨弹盘只允许负电流，正电流会导致反转。
    // 负电流最低限制为 -max_current。
    const float minimum_current = -max_current;
    const float limited = pid_output < minimum_current ? minimum_current : pid_output;
    return static_cast<int16_t>(limited);
}
