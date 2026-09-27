#include "ShooterTelemetryFormatter.hpp"

#include <cstdio>

std::size_t ShooterTelemetry_Format(char *buffer,
                                    const std::size_t buffer_size,
                                    const float friction_left_speed,
                                    const float friction_right_speed,
                                    const float feeder_target_position,
                                    const float feeder_position,
                                    const float feeder_speed,
                                    const float smc_output,
                                    const int16_t command_current,
                                    const bool stalled)
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return 0;
    }

    const int written = std::snprintf(
            buffer,
            buffer_size,
            "fricL=%.0f,fricR=%.0f,targetPos=%.2f,pos=%.2f,"
            "speed=%.2f,smc=%.0f,cmd=%d,stall=%d\r\n",
            friction_left_speed,
            friction_right_speed,
            feeder_target_position,
            feeder_position,
            feeder_speed,
            smc_output,
            command_current,
            stalled ? 1 : 0);

    if (written <= 0)
    {
        buffer[0] = '\0';
        return 0;
    }

    if (static_cast<std::size_t>(written) >= buffer_size)
    {
        return buffer_size - 1;
    }

    return static_cast<std::size_t>(written);
}
