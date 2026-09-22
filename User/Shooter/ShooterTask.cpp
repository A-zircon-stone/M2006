#include "ShooterController.hpp"
#include "ShooterMotors.hpp"
#include "ShooterRemote.hpp"

#include "RemoteC.h"
#include "FreeRTOS.h"
#include "task.h"

namespace
{
ShooterController shooter_controller;
bool last_fire_once_button = false;
}

extern "C" void ShooterTask(void const *argument)
{
    (void) argument;

    ShooterMotors_Init();
    shooter_controller.reset(ShooterMotors_ReadFeedback());

    TickType_t current = xTaskGetTickCount();
    for (;;)
    {
        const RC_ctrl_t remote = RC_GetDatas();
        const bool link_online = RC_GetNewData.live_time > 0;

        ShooterRemoteInput input;
        input.link_online = link_online;
        input.arm_switch_up = remote.rc.s[1] == RC_SW_UP;
        input.fire_hold = remote.mouse.press_l.Now_State != 0;
        input.fire_once_button = remote.key.R.Now_State != 0;

        const ShooterCommand command =
                BuildShooterCommand(input, last_fire_once_button);
        const ShooterFeedback feedback = ShooterMotors_ReadFeedback();

        shooter_controller.update(command, feedback);
        ShooterMotors_Update(shooter_controller.output());
        ShooterMotors_Send();

        // RC_GetNewData.live_time 由遥控接收中断刷新；这里负责超时倒计时。
        if (RC_GetNewData.live_time > 0)
        {
            --RC_GetNewData.live_time;
        }

        vTaskDelayUntil(&current, 1 / portTICK_RATE_MS);
    }
}
