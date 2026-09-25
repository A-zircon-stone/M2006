#include "RCPrintFormatter.hpp"

#include "DebugC.h"
#include "RemoteC.h"
#include "FreeRTOS.h"
#include "task.h"

namespace
{
constexpr TickType_t kPrintPeriod = 100 / portTICK_RATE_MS;
constexpr std::size_t kPrintBufferSize = 512;
char print_buffer[kPrintBufferSize]{};
}

extern "C" void RCPrintTestTask(void const *argument)
{
    (void) argument;

    TickType_t current = xTaskGetTickCount();
    for (;;)
    {
        RC_ctrl_t remote{};
        taskENTER_CRITICAL();
        remote = RC_GetDatas();
        taskEXIT_CRITICAL();

        const uint8_t switches[2] = {
                static_cast<uint8_t>(remote.rc.s[0]),
                static_cast<uint8_t>(remote.rc.s[1])
        };

        RCPrint_Format(print_buffer,
                       sizeof(print_buffer),
                       remote.rc.ch,
                       switches);

        usart_printf("%s", print_buffer);
        vTaskDelayUntil(&current, kPrintPeriod);
    }
}
