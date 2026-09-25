#include "RCPrintFormatter.hpp"

#include <cstdio>

std::size_t RCPrint_Format(char *buffer,
                           const std::size_t buffer_size,
                           const int16_t channels[5],
                           const uint8_t switches[2])
{
    if (buffer == nullptr || buffer_size == 0 || channels == nullptr || switches == nullptr)
    {
        return 0;
    }

    const int written = std::snprintf(
            buffer,
            buffer_size,
            "RC ch=[%d,%d,%d,%d,%d] s=[%u,%u]\r\n",
            channels[0], channels[1], channels[2], channels[3], channels[4],
            switches[0], switches[1]);

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
