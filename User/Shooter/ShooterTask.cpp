#include "FeederFireController.hpp"
#include "FeederPidSafety.hpp"
#include "FeederSmcController.hpp"
#include "FeederStallRecoveryController.hpp"
#include "FrictionReadyController.hpp"
#include "ShooterMotors.hpp"
#include "ShooterTelemetryFormatter.hpp"

#include "DebugC.h"
#include "RemoteC.h"
#include "FreeRTOS.h"
#include "task.h"

namespace
{
constexpr TickType_t kControlPeriod = 2 / portTICK_RATE_MS;
constexpr float kControlPeriodSeconds = 0.002f;
constexpr TickType_t kFeedbackSettleTime = 200 / portTICK_RATE_MS;
constexpr uint32_t kPrintDivider = 10;
constexpr std::size_t kPrintBufferSize = 200;

constexpr float kFrictionTargetSpeed = 6200.0f;
constexpr float kFrictionReadyMinimum = 5900.0f;
constexpr float kFrictionReadyMaximum = 6300.0f;
constexpr uint32_t kFrictionReadyCycles = 20;
constexpr bool kUseFrictionReadyCheck = true;

constexpr float kFeederSmcMaxCurrent = 9600.0f;
constexpr float kFeederSmcIntegralLimit = 2000.0f;
constexpr float kPositionStep = 45.0f;
constexpr float kPositionTolerance = 0.5f;
constexpr uint32_t kMinimumMotionCycles = 70;
constexpr uint32_t kPauseCycles = 10;

constexpr float kStallCurrentThreshold = 7500.0f;
constexpr uint32_t kStallDetectCycles = 100;
constexpr float kRecoveryStep = -45.0f;
constexpr uint32_t kRecoveryTimeoutCycles = 400;

char print_buffer[kPrintBufferSize]{};

void updateFrictionMotors(const bool enabled)
{
    if (!enabled)
    {
        shooter_fric_left.MotorCtrl.c_PID.SpdPIDClear();
        shooter_fric_right.MotorCtrl.c_PID.SpdPIDClear();
        shooter_fric_left.canSend(cMotor::eExternal, 0);
        shooter_fric_right.canSend(cMotor::eExternal, 0);
        return;
    }

    shooter_fric_left.MotorCtrl.c_PID.setSpdTar(-kFrictionTargetSpeed);
    shooter_fric_left.MotorCtrl.c_PID.spdLoop(
            shooter_fric_left.getEcd().speed);
    shooter_fric_left.canSend(cMotor::ePid);

    shooter_fric_right.MotorCtrl.c_PID.setSpdTar(kFrictionTargetSpeed);
    shooter_fric_right.MotorCtrl.c_PID.spdLoop(
            shooter_fric_right.getEcd().speed);
    shooter_fric_right.canSend(cMotor::ePid);
}

bool updateRemoteOnlineState()
{
    // 原云台任务每1 ms递减该计数；本最小工程停用了原任务，因此在这里接管。
    if (RC_GetNewData.live_time > 0)
    {
        --RC_GetNewData.live_time;
    }
    return RC_GetNewData.live_time > 0;
}
}

extern "C" void ShooterTask(void const *argument)
{
    (void) argument;

    ShooterMotors_Init();

    // 等待三台电机收到第一批 CAN 反馈，再把当前位置作为拨弹初始目标。
    vTaskDelay(kFeedbackSettleTime);
    Motor_measure_f feeder_feedback = shooter_feeder.getEcd();
    float target_position = feeder_feedback.total_angle;

    FeederSmcController feeder_smc(0.3f,
                                   2000.0f,
                                   2.0f,
                                   10.0f,
                                   1.1f,
                                   0.5f,
                                   kFeederSmcMaxCurrent,
                                   kFeederSmcIntegralLimit);
    feeder_smc.reset(target_position);

    FrictionReadyController friction_ready(kFrictionReadyMinimum,
                                           kFrictionReadyMaximum,
                                           kFrictionReadyCycles);
    FeederFireController fire_controller(kPositionStep,
                                         kPositionTolerance,
                                         kMinimumMotionCycles,
                                         kPauseCycles);
    FeederStallRecoveryController stall_recovery(
            kStallCurrentThreshold,
            kStallDetectCycles,
            kRecoveryStep,
            kRecoveryTimeoutCycles);

    const RC_ctrl_t initial_remote = RC_GetDatas();
    fire_controller.reset(target_position,
                          initial_remote.rc.ch[3] > 600);

    uint32_t print_count = 0;
    TickType_t current = xTaskGetTickCount();

    for (;;)
    {
        const RC_ctrl_t remote = RC_GetDatas();
        feeder_feedback = shooter_feeder.getEcd();

        const bool remote_online = updateRemoteOnlineState();
        const bool stop_requested =
                !remote_online || remote.rc.s[1] == RC_SW_UP;
        const bool friction_enabled = !stop_requested;

        updateFrictionMotors(friction_enabled);
        const bool fire_permitted = friction_ready.update(
                friction_enabled,
                shooter_fric_left.getEcd().speed,
                shooter_fric_right.getEcd().speed,
                kUseFrictionReadyCheck);

        bool force_zero_current = false;
        bool clear_integral = false;
        bool reset_smc = false;
        bool stall_active = false;

        if (stop_requested)
        {
            // S1=1或遥控离线：三台电机立即停止，并清除未完成的发射/反拨状态。
            friction_ready.reset();
            stall_recovery.reset();
            target_position = feeder_feedback.total_angle;
            fire_controller.reset(target_position,
                                  remote.rc.ch[3] > 600);
            reset_smc = true;
            force_zero_current = true;
        }
        else if (!fire_permitted)
        {
            // 与重构版一致：摩擦轮未稳定时不保持未完成的拨弹目标。
            stall_recovery.reset();
            target_position = feeder_feedback.total_angle;
            fire_controller.reset(target_position,
                                  remote.rc.ch[3] > 600);
            reset_smc = true;
            force_zero_current = true;
        }
        else
        {
            FeederStallRecoveryOutput recovery_output{};
            if (fire_controller.stepInProgress())
            {
                recovery_output = stall_recovery.update(
                        feeder_feedback.torque_current,
                        feeder_feedback.total_angle);
            }
            else
            {
                // 与同学版本一致：只在正常拨弹前进阶段累计堵转周期。
                stall_recovery.reset();
            }

            if (recovery_output.stalled)
            {
                // 堵转时暂停正常单发/连发，反向退一个齿位并持续清积分。
                stall_active = true;
                target_position = recovery_output.target_position;
                clear_integral = recovery_output.clear_integral;
            }
            else if (recovery_output.recovery_completed)
            {
                // 解卡结束同步实际位置并重置SMC；下周期再接受发射命令。
                target_position = feeder_feedback.total_angle;
                fire_controller.reset(target_position,
                                       remote.rc.ch[3] > 600);
                reset_smc = recovery_output.clear_integral;
            }
            else
            {
                const FeederFireOutput fire_output = fire_controller.update(
                        remote.rc.s[1],
                        remote.rc.s[0],
                        remote.rc.ch[3],
                        feeder_feedback.total_angle,
                        fire_permitted);
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

        const float smc_output = feeder_smc.update(
                target_position,
                feeder_feedback.total_angle,
                feeder_feedback.speed,
                kControlPeriodSeconds);
        const int16_t feeder_current = force_zero_current
                ? 0
                : FeederSmc_LimitBidirectionalCurrent(
                        smc_output,
                        kFeederSmcMaxCurrent);

        shooter_feeder.canSend(cMotor::eExternal, feeder_current);

        // 0x201、0x202、0x204同属CAN1的0x200控制帧，每周期只发送一次。
        CanReg[1].sendBuff();

        if (++print_count >= kPrintDivider)
        {
            print_count = 0;
            ShooterTelemetry_Format(
                    print_buffer,
                    sizeof(print_buffer),
                    shooter_fric_left.getEcd().speed,
                    shooter_fric_right.getEcd().speed,
                    target_position,
                    feeder_feedback.total_angle,
                    feeder_feedback.speed,
                    smc_output,
                    feeder_current,
                    stall_active);
            usart_printf("%s", print_buffer);
        }

        vTaskDelayUntil(&current, kControlPeriod);
    }
}
