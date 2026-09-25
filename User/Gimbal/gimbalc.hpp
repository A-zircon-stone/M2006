//
// Created by Andrew on 2024/8/27.
//

#ifndef FINALEXAM_GIMBAL_H
#define FINALEXAM_GIMBAL_H

#include "framework_headfile.hpp"
#include "../core.h"
#include "../MotorC.hpp"
#include "Shootc.hpp"
#include "IMUC.hpp"
#include "RC_Key.h"
#include "LEDC.h"
#include "sineware.hpp"


extern CanType CanReg[];


void Gimbal_Init();

void GimbalLoop();
void PrintTest();
/**com define**/
#define CHASSIS_VEL CanReg[6]
#define CHASSIS_YAW CanReg[7]
#define TIMpiece    0.001
#define PITCH_MAX 25
#define PITCH_MIN -10

/**external value**/
extern float G_offset;
/** Motor List **/
extern cMotor yaw, pitch_R, pitch_L,yaw_v2;   //R-0X206 R->无人机pitch,L-0X207
extern cMotor fricR, fricL;
extern cMotor rammc;
extern Shootc shoot;
extern cCar Car;

static float mouse_sense = TIMpiece * 2.5;  //鼠标灵敏度

float portSetYaw();
float Pitch_ecd_static_offset();
float portSetPitch();
void setSpin(float spinVel);

inline static SineWaveGenerator yawSineWave = SineWaveGenerator(20.0, 180, 0.0); // 创建一个正弦波生成器
inline static SineWaveGenerator pitchSineWave = SineWaveGenerator(1, 9, 6); // 创建一个正弦波生成器


float Graverity_Offet(float angle);

#endif //FINALEXAM_GIMBAL_H
