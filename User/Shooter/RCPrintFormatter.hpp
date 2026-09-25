#pragma once

#include <cstddef>
#include <cstdint>

std::size_t RCPrint_Format(char *buffer,
                           std::size_t buffer_size,
                           const int16_t channels[5],
                           const uint8_t switches[2]);
