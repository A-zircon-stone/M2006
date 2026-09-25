#include "RCPrintFormatter.hpp"

#include <cassert>
#include <cstring>

int main()
{
    char output[512]{};
    const int16_t channels[5] = {-660, -330, 0, 330, 660};
    const uint8_t switches[2] = {1, 3};

    const std::size_t length = RCPrint_Format(output,
                                               sizeof(output),
                                               channels,
                                               switches);

    const char expected[] = "RC ch=[-660,-330,0,330,660] s=[1,3]\r\n";

    assert(length == std::strlen(expected));
    assert(std::strcmp(output, expected) == 0);
    return 0;
}
