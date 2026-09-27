#pragma once

#include <cstdint>

class FrictionReadyController
{
public:
    FrictionReadyController(float minimum_speed,
                            float maximum_speed,
                            uint32_t required_cycles);

    bool update(bool friction_enabled,
                float left_speed,
                float right_speed,
                bool speed_check_enabled = true);
    bool ready() const;
    void reset();

private:
    float minimum_speed_;
    float maximum_speed_;
    uint32_t required_cycles_;
    uint32_t ready_cycles_;
    bool ready_;
};
