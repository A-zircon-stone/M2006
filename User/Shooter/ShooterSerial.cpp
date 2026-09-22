#include "ShooterSerial.hpp"

#include "usart.h"

#include <cctype>
#include <cstring>

namespace
{
constexpr std::size_t kLineCapacity = 32;
// 手动串口测试需要留出输入下一条命令的时间，同时保留断联保护。
constexpr uint32_t kCommandTimeoutMs = 5000;

char line[kLineCapacity]{};
std::size_t line_length = 0;
ShooterCommand command{};
bool fire_pending = false;
uint32_t last_command_tick = 0;

void stopCommand()
{
    command.armed = false;
    command.friction_on = false;
    command.fire_hold = false;
    command.emergency_stop = true;
    fire_pending = false;
}

void acceptLine()
{
    line[line_length] = '\0';

    if (std::strcmp(line, "FIRE") == 0)
    {
        command.emergency_stop = false;
        fire_pending = true;
    }
    else if (std::strcmp(line, "STOP") == 0)
    {
        stopCommand();
    }
    else if (std::strcmp(line, "ARM 1") == 0)
    {
        command.armed = true;
        command.friction_on = true;
        command.emergency_stop = false;
    }
    else if (std::strcmp(line, "ARM 0") == 0)
    {
        command.armed = false;
        command.friction_on = false;
    }
    else if (std::strcmp(line, "HOLD 1") == 0)
    {
        command.armed = true;
        command.friction_on = true;
        command.fire_hold = true;
        command.emergency_stop = false;
    }
    else if (std::strcmp(line, "HOLD 0") == 0)
    {
        command.fire_hold = false;
    }
    else
    {
        line_length = 0;
        return;
    }

    last_command_tick = HAL_GetTick();
    line_length = 0;
}

void pollUart()
{
    while (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_RXNE) != RESET)
    {
        const char received = static_cast<char>(huart6.Instance->DR & 0xFFU);

        if (received == '\r' || received == '\n')
        {
            if (line_length > 0)
            {
                acceptLine();
            }
        }
        else if (std::isprint(static_cast<unsigned char>(received)))
        {
            if (line_length < kLineCapacity - 1)
            {
                line[line_length++] = received;
            }
            else
            {
                line_length = 0;
            }
        }
    }
}
}

void ShooterSerial_Init()
{
    line_length = 0;
    command = ShooterCommand{};
    stopCommand();
    last_command_tick = HAL_GetTick();
}

ShooterCommand ShooterSerial_GetCommand()
{
    pollUart();

    if (HAL_GetTick() - last_command_tick > kCommandTimeoutMs)
    {
        stopCommand();
    }

    ShooterCommand output = command;
    output.fire_once = fire_pending;
    fire_pending = false;
    return output;
}
