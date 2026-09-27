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
    FeederStallRecoveryController(float stall_current_threshold,
                                  uint32_t stall_detect_cycles,
                                  float recovery_step,
                                  uint32_t recovery_timeout_cycles);

    FeederStallRecoveryOutput update(float feedback_current,
                                     float current_position);

    void reset();

private:
    float stall_current_threshold_;
    uint32_t stall_detect_cycles_;
    float recovery_step_;
    uint32_t recovery_timeout_cycles_;
    uint32_t stall_count_;
    uint32_t recovery_count_;
    float recovery_target_;
    bool stalled_;
};
