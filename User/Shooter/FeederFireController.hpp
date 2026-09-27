#pragma once

#include <cstdint>

struct FeederFireOutput
{
    float target_position;
    bool target_changed;
    bool step_in_progress;
    bool stop;
    bool shot_completed;
};

class FeederFireController
{
public:
    FeederFireController(float position_step,
                         float position_tolerance,
                         uint32_t minimum_motion_cycles,
                         uint32_t pause_cycles);

    void reset(float current_position, bool trigger_active);

    FeederFireOutput update(uint8_t s1,
                            uint8_t s2,
                            int16_t left_stick_vertical,
                            float current_position,
                            bool fire_permitted);

    bool stepInProgress() const { return step_in_progress_; }

private:
    float position_step_;
    float position_tolerance_;
    uint32_t minimum_motion_cycles_;
    uint32_t pause_cycles_;
    float target_position_;
    uint32_t motion_count_;
    uint32_t pause_count_;
    bool step_in_progress_;
    bool last_trigger_;
};
