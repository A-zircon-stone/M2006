#include "FeederStallRecoveryController.hpp"

#include <cmath>
#include <cstdlib>

FeederStallRecoveryController::FeederStallRecoveryController(
        const int16_t stall_current_threshold,
        const float stall_speed_threshold,
        const uint32_t stall_detect_cycles,
        const float recovery_step,
        const uint32_t recovery_timeout_cycles)
    : stall_current_threshold_(stall_current_threshold),
      stall_speed_threshold_(stall_speed_threshold),
      stall_detect_cycles_(stall_detect_cycles),
      recovery_step_(recovery_step),
      recovery_timeout_cycles_(recovery_timeout_cycles),
      stall_count_(0),
      recovery_count_(0),
      recovery_target_(0.0f),
      stalled_(false)
{
}

FeederStallRecoveryOutput FeederStallRecoveryController::update(
        const int16_t command_current,
        const float motor_speed,
        const float current_position)
{
    if (stalled_)
    {
        ++recovery_count_;

        const bool reached_target = recovery_step_ >= 0.0f
                ? current_position >= recovery_target_
                : current_position <= recovery_target_;
        const bool timed_out =
                recovery_count_ >= recovery_timeout_cycles_;

        if (reached_target || timed_out)
        {
            stalled_ = false;
            stall_count_ = 0;
            recovery_count_ = 0;
            return {false, true, true, current_position};
        }

        return {true, false, true, recovery_target_};
    }

    const bool current_is_saturated =
            std::abs(static_cast<int>(command_current)) >=
            stall_current_threshold_;
    const bool speed_is_low =
            std::fabs(motor_speed) <= stall_speed_threshold_;

    if (current_is_saturated && speed_is_low)
    {
        ++stall_count_;
    }
    else
    {
        stall_count_ = 0;
    }

    if (stall_count_ >= stall_detect_cycles_)
    {
        stalled_ = true;
        stall_count_ = 0;
        recovery_count_ = 0;
        recovery_target_ = current_position + recovery_step_;
        return {true, false, true, recovery_target_};
    }

    return {false, false, false, current_position};
}

void FeederStallRecoveryController::reset()
{
    stall_count_ = 0;
    recovery_count_ = 0;
    recovery_target_ = 0.0f;
    stalled_ = false;
}
