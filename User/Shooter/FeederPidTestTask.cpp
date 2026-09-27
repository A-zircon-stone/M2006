#include "FeederTelemetryFormatter.hpp"
#include "FeederPidSafety.hpp"
#include "FeederFireController.hpp"
#include "FeederSmcController.hpp"
#include "FeederStallRecoveryController.hpp"

#include "DebugC.h"
#include "MotorC.hpp"
#include "RemoteC.h"
#include "bsp_can.hpp"
#include "FreeRTOS.h"
#include "task.h"

namespace
{
constexpr TickType_t kControlPeriod = 2 / portTICK_RATE_MS;
constexpr float kControlPeriodSeconds = 0.002f;
constexpr uint32_t kPrintDivider = 10;
constexpr TickType_t kFeedbackSettleTime = 200 / portTICK_RATE_MS;
constexpr float kSmcMaxCurrent = 9600.0f;
constexpr float kSmcIntegralLimit = 2000.0f;
constexpr float kPositionStep = 45.0f;
constexpr float kPositionTolerance = 0.5f;
constexpr uint32_t kMinimumMotionCycles = 70;
constexpr uint32_t kPauseCycles = 10;
constexpr float kStallCurrentThreshold = 7500.0f;
constexpr uint32_t kStallDetectCycles = 100;
constexpr float kRecoveryStep = -45.0f;
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
    // 等待 CAN 反馈更新，再以当前输出轴位置作为初始目标。
    vTaskDelay(kFeedbackSettleTime);
    const Motor_measure_f initial_feedback = shooter_feeder.getEcd();
    float target_position = initial_feedback.total_angle;
    FeederSmcController feeder_smc(0.3f,
                                   2000.0f,
                                   2.0f,
                                   10.0f,
                                   1.1f,
                                   0.5f,
                                   kSmcMaxCurrent,
                                   kSmcIntegralLimit);
    feeder_smc.reset(target_position);

    FeederFireController fire_controller(kPositionStep,
                                         kPositionTolerance,
                                         kMinimumMotionCycles,
                                         kPauseCycles);
    const RC_ctrl_t initial_remote = RC_GetDatas();
    fire_controller.reset(target_position,
                          initial_remote.rc.ch[3] > 600);
    FeederStallRecoveryController stall_recovery(
            kStallCurrentThreshold,
            kStallDetectCycles,
            kRecoveryStep,
            kRecoveryTimeoutCycles);

    uint32_t print_count = 0;
    TickType_t current = xTaskGetTickCount();

    for (;;)
    {
        const Motor_measure_f feedback = shooter_feeder.getEcd();
        const RC_ctrl_t remote = RC_GetDatas();

        const bool stop_requested = remote.rc.s[1] == 1;
        bool force_zero_current = false;
        bool clear_integral = false;
        bool reset_smc = false;

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
            reset_smc = true;
            force_zero_current = true;
        }
        else
        {
            FeederStallRecoveryOutput recovery_output{};
            if (fire_controller.stepInProgress())
            {
                recovery_output = stall_recovery.update(
                        feedback.torque_current,
                        feedback.total_angle);
            }
            else
            {
                stall_recovery.reset();
            }

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
                reset_smc = recovery_output.clear_integral;
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
                clear_integral = fire_output.shot_completed;
            }
        }

        if (reset_smc)
        {
            feeder_smc.reset(target_position);
        }
        else if (clear_integral)
        {
            feeder_smc.clearIntegral();
        }

        const float smc_output = feeder_smc.update(target_position,
                                                   feedback.total_angle,
                                                   feedback.speed,
                                                   kControlPeriodSeconds);
        const float sliding_surface = feeder_smc.surface();
        const int16_t command_current = force_zero_current
                ? 0
                : FeederSmc_LimitBidirectionalCurrent(smc_output,
                                                      kSmcMaxCurrent);
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
