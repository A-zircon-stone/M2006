#include "FeederSmcController.hpp"

#include <algorithm>
#include <cmath>

FeederSmcController::FeederSmcController(const float inertia,
                                         const float reaching_gain,
                                         const float position_gain,
                                         const float integral_gain,
                                         const float saturation_scale,
                                         const float boundary_layer,
                                         const float max_current,
                                         const float integral_limit)
    : inertia_(inertia),
      reaching_gain_(reaching_gain),
      position_gain_(position_gain),
      integral_gain_(integral_gain),
      saturation_scale_(saturation_scale),
      boundary_layer_(boundary_layer),
      max_current_(max_current),
      integral_limit_(integral_limit)
{
}

void FeederSmcController::reset(const float target_position)
{
    last_target_position_ = target_position;
    initialized_ = true;
    target_speed_rpm_ = 0.0f;
    position_error_integral_ = 0.0f;
    surface_ = 0.0f;
}

void FeederSmcController::clearIntegral()
{
    position_error_integral_ = 0.0f;
}

float FeederSmcController::update(const float target_position,
                                  const float current_position,
                                  const float current_speed_rpm,
                                  const float dt_seconds)
{
    if (!std::isfinite(target_position) ||
        !std::isfinite(current_position) ||
        !std::isfinite(current_speed_rpm) ||
        !std::isfinite(dt_seconds) || dt_seconds <= 0.0f)
    {
        target_speed_rpm_ = 0.0f;
        surface_ = 0.0f;
        return 0.0f;
    }

    if (!initialized_)
    {
        reset(target_position);
    }

    // 目标位置单位为度，电机速度反馈单位为rpm：deg/s除以6得到rpm。
    target_speed_rpm_ =
            (target_position - last_target_position_) / dt_seconds / 6.0f;
    last_target_position_ = target_position;

    const float position_error = current_position - target_position;
    const float speed_error = current_speed_rpm - target_speed_rpm_;
    const float next_integral = std::clamp(
            position_error_integral_ + position_error * dt_seconds,
            -integral_limit_,
            integral_limit_);

    float proposed_surface = 0.0f;
    const float proposed_output = calculate(position_error,
                                            speed_error,
                                            next_integral,
                                            &proposed_surface);

    // 使用本周期预计输出做抗饱和判断；达到限幅时不接受本次积分。
    if (std::fabs(proposed_output) < max_current_)
    {
        position_error_integral_ = next_integral;
    }

    const float output = calculate(position_error,
                                   speed_error,
                                   position_error_integral_,
                                   &surface_);
    return std::clamp(output, -max_current_, max_current_);
}

float FeederSmcController::calculate(const float position_error,
                                     const float speed_error,
                                     const float position_error_integral,
                                     float *const surface) const
{
    const float sliding_surface =
            position_gain_ * position_error +
            speed_error +
            integral_gain_ * position_error_integral;
    if (surface != nullptr)
    {
        *surface = sliding_surface;
    }

    float saturation = 0.0f;
    if (saturation_scale_ > 0.0f)
    {
        const float scaled = sliding_surface / saturation_scale_;
        if (std::fabs(scaled) <= boundary_layer_)
        {
            saturation = scaled;
        }
        else
        {
            saturation = scaled > 0.0f ? 1.0f : -1.0f;
        }
    }

    return inertia_ *
           (-position_gain_ * speed_error
            -integral_gain_ * position_error
            -reaching_gain_ * sliding_surface
            -saturation_scale_ * saturation);
}

float FeederSmcController::surface() const
{
    return surface_;
}

float FeederSmcController::targetSpeedRpm() const
{
    return target_speed_rpm_;
}

float FeederSmcController::positionErrorIntegral() const
{
    return position_error_integral_;
}
