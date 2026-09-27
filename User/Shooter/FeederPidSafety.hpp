#pragma once

#include <cstdint>

int16_t FeederPid_LimitForwardCurrent(float pid_output, float max_current);

int16_t FeederSmc_LimitBidirectionalCurrent(float smc_output,
                                            float max_current);
