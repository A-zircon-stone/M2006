#include "FeederModeController.hpp"

namespace
{
constexpr uint8_t kSwitchUp = 1;
constexpr uint8_t kSwitchDown = 2;
constexpr int16_t kTriggerThreshold = 600;
}

FeederModeController::FeederModeController(
        const float continuous_speed_target)
    : continuous_speed_target_(continuous_speed_target),
      continuous_active_(false)
{
}

FeederModeOutput FeederModeController::update(
        const uint8_t s1,
        const uint8_t s2,
        const int16_t left_stick_vertical)
{
    const bool stop = s1 == kSwitchUp;
    const bool continuous_requested =
            !stop && s2 == kSwitchDown &&
            left_stick_vertical > kTriggerThreshold;

    const bool entered_continuous =
            continuous_requested && !continuous_active_;
    const bool left_continuous =
            !continuous_requested && continuous_active_;
    continuous_active_ = continuous_requested;

    if (stop)
    {
        return {FeederControlMode::Stop,
                0.0f,
                false,
                left_continuous};
    }

    if (continuous_requested)
    {
        return {FeederControlMode::ContinuousSpeed,
                continuous_speed_target_,
                entered_continuous,
                false};
    }

    return {FeederControlMode::Position,
            0.0f,
            false,
            left_continuous};
}
