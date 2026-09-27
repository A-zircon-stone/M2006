#pragma once

#include <cstdint>

enum class FeederControlMode
{
    Stop,
    Position,
    ContinuousSpeed
};

struct FeederModeOutput
{
    FeederControlMode mode;
    float continuous_speed_target;
    bool entered_continuous;
    bool left_continuous;
};

class FeederModeController
{
public:
    explicit FeederModeController(float continuous_speed_target);

    FeederModeOutput update(uint8_t s1,
                            uint8_t s2,
                            int16_t left_stick_vertical);

private:
    float continuous_speed_target_;
    bool continuous_active_;
};
