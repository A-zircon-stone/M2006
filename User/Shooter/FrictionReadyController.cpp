#include "FrictionReadyController.hpp"

FrictionReadyController::FrictionReadyController(
        const float minimum_speed,
        const float maximum_speed,
        const uint32_t required_cycles)
    : minimum_speed_(minimum_speed),
      maximum_speed_(maximum_speed),
      required_cycles_(required_cycles),
      ready_cycles_(0),
      ready_(false)
{
}

bool FrictionReadyController::update(const bool friction_enabled,
                                     const float left_speed,
                                     const float right_speed)
{
    if (!friction_enabled)
    {
        reset();
        return false;
    }

    const bool left_in_range =
            left_speed >= -maximum_speed_ && left_speed <= -minimum_speed_;
    const bool right_in_range =
            right_speed >= minimum_speed_ && right_speed <= maximum_speed_;

    if (!left_in_range || !right_in_range)
    {
        reset();
        return false;
    }

    if (ready_cycles_ < required_cycles_)
    {
        ++ready_cycles_;
    }
    ready_ = ready_cycles_ >= required_cycles_;
    return ready_;
}

bool FrictionReadyController::ready() const
{
    return ready_;
}

void FrictionReadyController::reset()
{
    ready_cycles_ = 0;
    ready_ = false;
}
