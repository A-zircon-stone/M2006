#include "ShooterMotors.hpp"

// CAN 反馈 ID 与发送组由 cMotor::canInit() 根据电机型号自动映射。
cMotor shooter_fric_left;
cMotor shooter_fric_right;
cMotor shooter_feeder;

namespace
{
void initFrictionMotor(cMotor &motor)
{
    motor.MotorCtrl.c_PID.Init();
    motor.MotorCtrl.c_PID.setParam(20.0f, 0.0f, 0.0f, 16384.0f,
                                   20.0f, 0.0f, 0.0f, 400.0f);
}

void initFeederMotor(cMotor &motor)
{
    motor.MotorCtrl.c_PID.Init();
    motor.MotorCtrl.c_PID.setParam(0.0f, 0.0f, 0.0f, 10000.0f,
                                   0.0f, 0.0f, 0.0f, 500.0f);
    motor.MotorCtrl.c_SMC.Init();
    motor.MotorCtrl.c_SMC.SetParam(0.3f, 2000.0f, 2.0f, 10.0f,
                                   1.1f, 0.5f, 9600.0f, EISMC, 0.0f);
}

void stopMotor(cMotor &motor)
{
    motor.MotorCtrl.c_PID.SpdPIDClear();
    motor.MotorCtrl.c_SMC.Clear();
    motor.canSend(cMotor::eExternal, 0);
}
}

void ShooterMotors_Init()
{
    shooter_fric_left.canInit(0x201, &hcan1, cMotor::M_3508);
    shooter_fric_right.canInit(0x202, &hcan1, cMotor::M_3508);
    shooter_feeder.canInit(0x203, &hcan1, cMotor::M_2006_p36);

    initFrictionMotor(shooter_fric_left);
    initFrictionMotor(shooter_fric_right);
    initFeederMotor(shooter_feeder);
}

ShooterFeedback ShooterMotors_ReadFeedback()
{
    ShooterFeedback feedback;
    feedback.left_speed = shooter_fric_left.getEcd().speed;
    feedback.right_speed = shooter_fric_right.getEcd().speed;
    feedback.feeder_position = shooter_feeder.getEcd().total_angle;
    return feedback;
}

void ShooterMotors_Update(const ShooterOutput &output)
{
    if (output.safe_output)
    {
        stopMotor(shooter_fric_left);
        stopMotor(shooter_fric_right);
        stopMotor(shooter_feeder);
        return;
    }

    shooter_fric_left.MotorCtrl.c_PID.setSpdTar(output.left_speed_target);
    shooter_fric_left.MotorCtrl.c_PID.spdLoop(shooter_fric_left.getEcd().speed);
    shooter_fric_left.canSend(cMotor::ePid);

    shooter_fric_right.MotorCtrl.c_PID.setSpdTar(output.right_speed_target);
    shooter_fric_right.MotorCtrl.c_PID.spdLoop(shooter_fric_right.getEcd().speed);
    shooter_fric_right.canSend(cMotor::ePid);

    shooter_feeder.MotorCtrl.c_SMC.ErrorUpdate(output.feeder_target,
                                                shooter_feeder.getEcd().total_angle,
                                                shooter_feeder.getEcd().speed);
    shooter_feeder.MotorCtrl.c_SMC.SmcCalculate();
    shooter_feeder.canSend(cMotor::eExternal,
                           static_cast<int16_t>(shooter_feeder.MotorCtrl.c_SMC.Out()));
}

void ShooterMotors_Send()
{
    // 三台发射电机对应同一个 DJI 0x200 电流控制帧。
    CanReg[1].sendBuff();
}
