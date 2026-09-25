#include "FeederTelemetryFormatter.hpp"
#include "FeederPidSafety.hpp"
#include "FeederFireController.hpp"

#include "DebugC.h"
#include "MotorC.hpp"
#include "RemoteC.h"
#include "bsp_can.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace
{
constexpr TickType_t kControlPeriod = 1 / portTICK_RATE_MS;
constexpr uint32_t kPrintDivider = 20;
constexpr TickType_t kFeedbackSettleTime = 200 / portTICK_RATE_MS;
// 速度内环沿用当前已经调好的参数。
constexpr float kSpeedKp = 20.0f;
constexpr float kSpeedKi = 2.0f;
constexpr float kSpeedKd = 0.0f;
constexpr float kMaxCurrent = 3000.0f;
// 第一轮位置外环只使用 P，位置环输出限制为目标速度 [-30, 30]。
constexpr float kPositionKp = 4.0f;
constexpr float kPositionKi = 0.0f;
constexpr float kPositionKd = 0.0f;
constexpr float kMaxTargetSpeed = 20.0f;
constexpr float kPositionStep = -30.0f;
constexpr float kPositionTolerance = 1.0f;
constexpr uint32_t kShotIntervalCycles = 30;
constexpr std::size_t kPrintBufferSize = 160;

char print_buffer[kPrintBufferSize]{};
}

extern cMotor shooter_feeder;

extern "C" void FeederPidTestTask(void const *argument)
{
    (void) argument;

    // 当前调 M2006 位置-速度双环；正式发射任务仍保持停用。
    shooter_feeder.canInit(0x204, &hcan1, cMotor::M_2006_p36);
    shooter_feeder.MotorCtrl.c_PID.Init();
    shooter_feeder.MotorCtrl.c_PID.setParam(kSpeedKp,
                                            kSpeedKi,
                                            kSpeedKd,
                                            kMaxCurrent,
                                            kPositionKp,
                                            kPositionKi,
                                            kPositionKd,
                                            kMaxTargetSpeed);

    // 等待 CAN 反馈更新，再以当前输出轴位置作为初始目标。
    vTaskDelay(kFeedbackSettleTime);
    float target_position = shooter_feeder.getEcd().total_angle;
    shooter_feeder.MotorCtrl.c_PID.setPosTar(target_position);
    FeederFireController fire_controller(kPositionStep,
                                         kPositionTolerance,
                                         kShotIntervalCycles);
    const RC_ctrl_t initial_remote = RC_GetDatas();
    fire_controller.reset(target_position,
                          initial_remote.rc.ch[3] > 600);

    uint32_t print_count = 0;
    TickType_t current = xTaskGetTickCount();

    for (;;)
    {
        const Motor_measure_f feedback = shooter_feeder.getEcd();
        const RC_ctrl_t remote = RC_GetDatas();

        const FeederFireOutput fire_output =
                fire_controller.update(remote.rc.s[1],
                                       remote.rc.s[0],
                                       remote.rc.ch[3],
                                       feedback.total_angle);
        target_position = fire_output.target_position;
        if (fire_output.target_changed)
        {
            shooter_feeder.MotorCtrl.c_PID.setPosTar(target_position);
        }

        // 位置环输出目标速度，速度环再根据实际速度输出电流。
        shooter_feeder.MotorCtrl.c_PID.posLoop(feedback.total_angle,
                                               feedback.speed);
        const float target_speed =
                shooter_feeder.MotorCtrl.c_PID.getSpdTar();
        const float pid_output = shooter_feeder.MotorCtrl.c_PID.pidOut();
        const int16_t command_current = fire_output.stop
                ? 0
                : FeederPid_LimitForwardCurrent(pid_output, kMaxCurrent);
        shooter_feeder.canSend(cMotor::eExternal, command_current);
        CanReg[1].sendBuff();

        if (++print_count >= kPrintDivider)
        {
            print_count = 0;

            FeederTelemetry_Format(print_buffer,
                                   sizeof(print_buffer),
                                   target_position,
                                   feedback.total_angle,
                                   target_speed,
                                   feedback.speed,
                                   pid_output,
                                   command_current,
                                   feedback.torque_current,
                                   feedback.temp);
            usart_printf("%s", print_buffer);
        }

        vTaskDelayUntil(&current, kControlPeriod);
    }
}
