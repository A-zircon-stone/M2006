#include "FeederTelemetryFormatter.hpp"

#include <cstdio>

std::size_t FeederTelemetry_Format(char *buffer,
                                   const std::size_t buffer_size,
                                   const float target_position,
                                   const float position,
                                   const float target_speed,
                                   const float speed,
                                   const float pid_output,
                                   const int16_t command_current,
                                   const float feedback_current,
                                   const float temperature)
{
    if (buffer == nullptr || buffer_size == 0)
    {
        return 0;
    }

    const int written = std::snprintf(buffer,
                                      buffer_size,
                                      "targetPos=%.2f,pos=%.2f,targetSpeed=%.2f,speed=%.2f,"
                                      "pid=%.0f,cmd=%d,current=%.0f,temp=%.0f\r\n",
                                      target_position,
                                      position,
                                      target_speed,
                                      speed,
                                      pid_output,
                                      command_current,
                                      feedback_current,
                                      temperature);

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
