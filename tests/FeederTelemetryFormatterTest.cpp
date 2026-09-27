#include "FeederTelemetryFormatter.hpp"

#include <cassert>
#include <cstring>

int main()
{
    char output[160]{};

    const std::size_t length = FeederTelemetry_Format(output,
                                                       sizeof(output),
                                                       441.78f,
                                                       456.78f,
                                                       -15.0f,
                                                       -32.5f,
                                                       -650.0f,
                                                       -650,
                                                       -620.0f,
                                                       48.0f);

    const char expected[] =
            "targetPos=441.78,pos=456.78,surface=-15.00,speed=-32.50,"
            "smc=-650,cmd=-650,current=-620,temp=48\r\n";

    assert(length == std::strlen(expected));
    assert(std::strcmp(output, expected) == 0);
    return 0;
}
