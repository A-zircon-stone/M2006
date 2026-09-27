#include "FeederFireController.hpp"

namespace
{
constexpr uint8_t kSwitchUp = 1;
constexpr uint8_t kSwitchDown = 2;
constexpr int16_t kTriggerThreshold = 600;
}

FeederFireController::FeederFireController(const float position_step,
                                           const float position_tolerance,
                                           const uint32_t minimum_motion_cycles,
                                           const uint32_t pause_cycles)
    : position_step_(position_step),
      position_tolerance_(position_tolerance),
      minimum_motion_cycles_(minimum_motion_cycles),
      pause_cycles_(pause_cycles),
      target_position_(0.0f),
      motion_count_(0),
      pause_count_(0),
      step_in_progress_(false),
      last_trigger_(false)
{
}

void FeederFireController::reset(const float current_position,
                                 const bool trigger_active)
{
    target_position_ = current_position;
    motion_count_ = 0;
    pause_count_ = 0;
    step_in_progress_ = false;
    last_trigger_ = trigger_active;
}

FeederFireOutput FeederFireController::update(const uint8_t s1,
                                              const uint8_t s2,
                                              const int16_t left_stick_vertical,
                                              const float current_position,
                                              const bool fire_permitted)
{
    const bool trigger = left_stick_vertical > kTriggerThreshold;
    const bool stop = s1 == kSwitchUp;

    if (stop)
    {
        reset(current_position, trigger);
        return {target_position_, true, false, true, false};
    }

    bool target_changed = false;
    bool shot_completed = false;

    const bool step_reached = position_step_ >= 0.0f
            ? current_position >= target_position_ - position_tolerance_
            : current_position <= target_position_ + position_tolerance_;

    if (step_in_progress_)
    {
        if (motion_count_ < minimum_motion_cycles_)
        {
            ++motion_count_;
        }

        if (motion_count_ >= minimum_motion_cycles_ && step_reached)
        {
            if (pause_count_ < pause_cycles_)
            {
                ++pause_count_;
            }

            if (pause_count_ >= pause_cycles_)
            {
                step_in_progress_ = false;
                target_position_ = current_position;
                target_changed = true;
                shot_completed = true;
            }
        }
    }

    const bool fire_once = s2 == kSwitchUp && trigger && !last_trigger_;
    const bool fire_hold = s2 == kSwitchDown && trigger;

    if (fire_permitted && !step_in_progress_ && !shot_completed &&
        (fire_once || fire_hold))
    {
        // 与原版发射逻辑一致：每一发都从此刻的实际位置生成目标。
        // 即使上一发存在少量过冲，也不会让理论目标与机械位置越积越远。
        target_position_ = current_position + position_step_;
        step_in_progress_ = true;
        motion_count_ = 0;
        pause_count_ = 0;
        target_changed = true;
    }

    last_trigger_ = trigger;
    return {target_position_, target_changed, step_in_progress_, false,
            shot_completed};
}
