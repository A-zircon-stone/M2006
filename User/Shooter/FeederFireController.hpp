#pragma once

#include <cstdint>

struct FeederFireOutput
{
    float target_position;
    bool target_changed;
    bool step_in_progress;
    bool stop;
};

class FeederFireController
{
public:
    FeederFireController(float position_step,
                         float position_tolerance,
                         uint32_t interval_cycles);

    void reset(float current_position, bool trigger_active);

    FeederFireOutput update(uint8_t s1,
                            uint8_t s2,
                            int16_t left_stick_vertical,
                            float current_position);

private:
    float position_step_;
    float position_tolerance_;
    uint32_t interval_cycles_;
    float target_position_;
    uint32_t interval_count_;
    bool step_in_progress_;
    bool last_trigger_;
};
