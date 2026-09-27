#include "FeederFireController.hpp"

namespace
{
constexpr uint8_t kSwitchUp = 1;
constexpr uint8_t kSwitchDown = 2;
constexpr int16_t kTriggerThreshold = 600;
}

FeederFireController::FeederFireController(const float position_step,
                                           const float position_tolerance,
                                           const uint32_t interval_cycles)
    : position_step_(position_step),
      position_tolerance_(position_tolerance),
      interval_cycles_(interval_cycles),
      target_position_(0.0f),
      interval_count_(0),
      step_in_progress_(false),
      last_trigger_(false)
{
}

void FeederFireController::reset(const float current_position,
                                 const bool trigger_active)
{
    target_position_ = current_position;
    interval_count_ = 0;
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
        return {target_position_, true, false, true};
    }

    bool target_changed = false;
    bool step_completed = false;

    const bool step_reached = position_step_ >= 0.0f
            ? current_position >= target_position_ - position_tolerance_
            : current_position <= target_position_ + position_tolerance_;

    if (step_in_progress_ && step_reached)
    {
        step_in_progress_ = false;
        interval_count_ = interval_cycles_;
        step_completed = true;
    }

    // 到位周期只进入发射间隔；从下一个控制周期起再开始倒计时。
    if (!step_completed && !step_in_progress_ && interval_count_ > 0)
    {
        --interval_count_;
    }

    const bool fire_once = s2 == kSwitchUp && trigger && !last_trigger_;
    const bool fire_hold = s2 == kSwitchDown && trigger;

    if (fire_permitted && !step_in_progress_ && interval_count_ == 0 &&
        (fire_once || fire_hold))
    {
        // 与原版发射逻辑一致：每一发都从此刻的实际位置生成目标。
        // 即使上一发存在少量过冲，也不会让理论目标与机械位置越积越远。
        target_position_ = current_position + position_step_;
        step_in_progress_ = true;
        target_changed = true;
    }

    last_trigger_ = trigger;
    return {target_position_, target_changed, step_in_progress_, false};
}
