#pragma once

#include <cstdint>

struct FeederStallRecoveryOutput
{
    bool stalled;
    bool recovery_completed;
    bool clear_integral;
    float target_position;
};

class FeederStallRecoveryController
{
public:
    FeederStallRecoveryController(int16_t stall_current_threshold,
                                  float stall_speed_threshold,
                                  uint32_t stall_detect_cycles,
                                  float recovery_step,
                                  uint32_t recovery_timeout_cycles);

    FeederStallRecoveryOutput update(int16_t command_current,
                                     float motor_speed,
                                     float current_position);

    void reset();

private:
    int16_t stall_current_threshold_;
    float stall_speed_threshold_;
    uint32_t stall_detect_cycles_;
    float recovery_step_;
    uint32_t recovery_timeout_cycles_;
    uint32_t stall_count_;
    uint32_t recovery_count_;
    float recovery_target_;
    bool stalled_;
};
