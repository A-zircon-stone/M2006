#pragma once

class FeederSmcController
{
public:
    FeederSmcController(float inertia,
                        float reaching_gain,
                        float position_gain,
                        float integral_gain,
                        float saturation_scale,
                        float boundary_layer,
                        float max_current,
                        float integral_limit);

    void reset(float target_position);
    void clearIntegral();

    float update(float target_position,
                 float current_position,
                 float current_speed_rpm,
                 float dt_seconds);

    float surface() const;
    float targetSpeedRpm() const;
    float positionErrorIntegral() const;

private:
    float calculate(float position_error,
                    float speed_error,
                    float position_error_integral,
                    float *surface) const;

    float inertia_;
    float reaching_gain_;
    float position_gain_;
    float integral_gain_;
    float saturation_scale_;
    float boundary_layer_;
    float max_current_;
    float integral_limit_;
    float last_target_position_ = 0.0f;
    bool initialized_ = false;
    float target_speed_rpm_ = 0.0f;
    float position_error_integral_ = 0.0f;
    float surface_ = 0.0f;
};
