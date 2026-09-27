#pragma once

#include <cstddef>
#include <cstdint>

std::size_t ShooterTelemetry_Format(char *buffer,
                                    std::size_t buffer_size,
                                    float friction_left_speed,
                                    float friction_right_speed,
                                    float feeder_target_position,
                                    float feeder_position,
                                    float feeder_speed,
                                    float smc_output,
                                    int16_t command_current,
                                    bool stalled);
