#include "ShooterController.hpp"//状态机
#include "ShooterMotors.hpp"//三台电机
#include "ShooterRemote.hpp"//遥控器输入转换
#include "ShooterSerial.hpp"//电脑串口输入

#include "RemoteC.h"//遥控器原始数据
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
//初始化
    ShooterMotors_Init();
    ShooterSerial_Init();
    shooter_controller.reset(ShooterMotors_ReadFeedback());

    TickType_t current = xTaskGetTickCount();
    for (;;)
    {
        // 当前使用电脑串口控制。每条命令以回车或换行结束。
        const ShooterCommand command = ShooterSerial_GetCommand();

        /*
        // 切回 DT7/DR16 时，注释掉上面的串口命令，解除本段注释。
        const RC_ctrl_t remote = RC_GetDatas();//读取输入();

        const bool link_online = RC_GetNewData.live_time > 0;

        ShooterRemoteInput input;
        input.link_online = link_online;
        input.arm_switch_up = remote.rc.s[1] == RC_SW_UP;
        input.fire_hold = remote.mouse.press_l.Now_State != 0;
        input.fire_once_button = remote.key.R.Now_State != 0;

        const ShooterCommand command =
                BuildShooterCommand(input, last_fire_once_button);
        */
        const ShooterFeedback feedback = ShooterMotors_ReadFeedback();//读取电机反馈

        shooter_controller.update(command, feedback);//更新状态机
        ShooterMotors_Update(shooter_controller.output());//计算电机输出
        ShooterMotors_Send();//发送电流值

        vTaskDelayUntil(&current, 1 / portTICK_RATE_MS);
    }
}
