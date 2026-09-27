#include "FeederTelemetryFormatter.hpp"
#include "FeederPidSafety.hpp"
#include "FeederFireController.hpp"
#include "FeederStallRecoveryController.hpp"

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
// 先沿用原车 EISMC 的结构参数，但把输出限制在较小范围内验证方向和制动。
constexpr float kSmcJ = 0.3f;
constexpr float kSmcK = 2000.0f;
constexpr float kSmcC1 = 2.0f;
constexpr float kSmcC2 = 10.0f;
constexpr float kSmcEpsilon = 1.1f;
constexpr float kSmcSaturationLimit = 0.5f;
constexpr float kSmcMaxCurrent = 5000.0f;
constexpr float kSmcPositionDeadband = 0.2f;
constexpr float kPositionStep = 40.0f;
constexpr float kPositionTolerance = 1.0f;
constexpr uint32_t kShotIntervalCycles = 30;
constexpr int16_t kStallCurrentThreshold = 1400;
constexpr float kStallSpeedThreshold = 5.0f;
constexpr uint32_t kStallDetectCycles = 100;
constexpr float kRecoveryStep = -20.0f;
constexpr uint32_t kRecoveryTimeoutCycles = 400;
constexpr std::size_t kPrintBufferSize = 160;

char print_buffer[kPrintBufferSize]{};
}

extern cMotor shooter_feeder;

extern "C" void FeederPidTestTask(void const *argument)
{
    (void) argument;

    // 当前验证 M2006 的 EISMC 位置控制；正式发射任务仍保持停用。
    shooter_feeder.canInit(0x204, &hcan1, cMotor::M_2006_p36);
    shooter_feeder.MotorCtrl.c_SMC.Init();
    shooter_feeder.MotorCtrl.c_SMC.SetParam(kSmcJ,
                                            kSmcK,
                                            kSmcC1,
                                            kSmcC2,
                                            kSmcEpsilon,
                                            kSmcSaturationLimit,
                                            kSmcMaxCurrent,
                                            EISMC,
                                            kSmcPositionDeadband);

    // 等待 CAN 反馈更新，再以当前输出轴位置作为初始目标。
    vTaskDelay(kFeedbackSettleTime);
    float target_position = shooter_feeder.getEcd().total_angle;

    // 先让 SMC 记住当前目标，避免首次更新把“0 到当前累计角度”误认为目标突变。
    const Motor_measure_f initial_feedback = shooter_feeder.getEcd();
    shooter_feeder.MotorCtrl.c_SMC.ErrorUpdate(target_position,
                                               initial_feedback.total_angle,
                                               initial_feedback.speed);
    shooter_feeder.MotorCtrl.c_SMC.ErrorUpdate(target_position,
                                               initial_feedback.total_angle,
                                               initial_feedback.speed);
    shooter_feeder.MotorCtrl.c_SMC.Integval_Clear();

    FeederFireController fire_controller(kPositionStep,
                                         kPositionTolerance,
                                         kShotIntervalCycles);
    const RC_ctrl_t initial_remote = RC_GetDatas();
    fire_controller.reset(target_position,
                          initial_remote.rc.ch[3] > 600);
    FeederStallRecoveryController stall_recovery(
            kStallCurrentThreshold,
            kStallSpeedThreshold,
            kStallDetectCycles,
            kRecoveryStep,
            kRecoveryTimeoutCycles);

    uint32_t print_count = 0;
    int16_t previous_command_current = 0;
    TickType_t current = xTaskGetTickCount();

    for (;;)
    {
        const Motor_measure_f feedback = shooter_feeder.getEcd();
        const RC_ctrl_t remote = RC_GetDatas();

        const bool stop_requested = remote.rc.s[1] == 1;
        bool force_zero_current = false;
        bool clear_integral = false;

        if (stop_requested)
        {
            // 人工停止优先级最高，立即取消自动反拨并同步到当前位置。
            stall_recovery.reset();
            const FeederFireOutput fire_output =
                    fire_controller.update(remote.rc.s[1],
                                           remote.rc.s[0],
                                           remote.rc.ch[3],
                                           feedback.total_angle,
                                           true);
            target_position = fire_output.target_position;
            clear_integral = true;
            force_zero_current = true;
        }
        else
        {
            const FeederStallRecoveryOutput recovery_output =
                    stall_recovery.update(previous_command_current,
                                          feedback.speed,
                                          feedback.total_angle);

            if (recovery_output.stalled)
            {
                // 解卡期间暂停正常发射目标，按原版做法每周期清积分。
                target_position = recovery_output.target_position;
                clear_integral = recovery_output.clear_integral;
            }
            else if (recovery_output.recovery_completed)
            {
                // 反拨到位或超时后先同步实际位置，本周期不立刻继续发射。
                target_position = feedback.total_angle;
                fire_controller.reset(target_position,
                                      remote.rc.ch[3] > 600);
                clear_integral = recovery_output.clear_integral;
                force_zero_current = true;
            }
            else
            {
                const FeederFireOutput fire_output =
                        fire_controller.update(remote.rc.s[1],
                                               remote.rc.s[0],
                                               remote.rc.ch[3],
                                               feedback.total_angle,
                                               true);
                target_position = fire_output.target_position;
            }
        }

        if (clear_integral)
        {
            shooter_feeder.MotorCtrl.c_SMC.Integval_Clear();
        }

        // SMC 同时使用位置误差和当前速度计算驱动/制动电流。
        shooter_feeder.MotorCtrl.c_SMC.ErrorUpdate(target_position,
                                                   feedback.total_angle,
                                                   feedback.speed);
        const float smc_output =
                shooter_feeder.MotorCtrl.c_SMC.SmcCalculate();
        const float sliding_surface =
                shooter_feeder.MotorCtrl.c_SMC.getSmc().s;
        const int16_t command_current = force_zero_current
                ? 0
                : FeederSmc_LimitBidirectionalCurrent(smc_output,
                                                      kSmcMaxCurrent);
        previous_command_current = command_current;
        shooter_feeder.canSend(cMotor::eExternal, command_current);
        CanReg[1].sendBuff();

        if (++print_count >= kPrintDivider)
        {
            print_count = 0;

            FeederTelemetry_Format(print_buffer,
                                   sizeof(print_buffer),
                                   target_position,
                                   feedback.total_angle,
                                   sliding_surface,
                                   feedback.speed,
                                   smc_output,
                                   command_current,
                                   feedback.torque_current,
                                   feedback.temp);
            usart_printf("%s", print_buffer);
        }

        vTaskDelayUntil(&current, kControlPeriod);
    }
}
