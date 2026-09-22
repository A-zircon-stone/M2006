#ifndef SHOOTER_MOTORS_HPP
#define SHOOTER_MOTORS_HPP

#include "MotorC.hpp"
#include "ShooterController.hpp"

// 发射机构的三台电机。对象名称带 shooter 前缀，避免和旧云台全局对象混淆。
extern cMotor shooter_fric_left;
extern cMotor shooter_fric_right;
extern cMotor shooter_feeder;

void ShooterMotors_Init();
ShooterFeedback ShooterMotors_ReadFeedback();
void ShooterMotors_Update(const ShooterOutput &output);

#endif
