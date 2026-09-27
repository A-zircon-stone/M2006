#include "ShooterTelemetryFormatter.hpp"

#include <cassert>
#include <cstring>

int main()
{
    char output[200]{};

    const std::size_t length = ShooterTelemetry_Format(output,
                                                        sizeof(output),
                                                        -6180.0f,
                                                        6210.0f,
                                                        140.0f,
                                                        125.3f,
                                                        110.0f,
                                                        1260.0f,
                                                        1260,
                                                        false);

    const char expected[] =
            "fricL=-6180,fricR=6210,targetPos=140.00,pos=125.30,"
            "speed=110.00,smc=1260,cmd=1260,stall=0\r\n";

    assert(length == std::strlen(expected));
    assert(std::strcmp(output, expected) == 0);
    return 0;
}
