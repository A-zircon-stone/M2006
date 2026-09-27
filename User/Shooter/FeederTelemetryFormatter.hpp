#pragma once

#include <cstddef>
#include <cstdint>

std::size_t FeederTelemetry_Format(char *buffer,
                                   std::size_t buffer_size,
                                   float target_position,
                                   float position,
                                   float sliding_surface,
                                   float speed,
                                   float controller_output,
                                   int16_t command_current,
                                   float feedback_current,
                                   float temperature);
