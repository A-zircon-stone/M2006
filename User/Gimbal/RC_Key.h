//
// Created by hx on 2024/9/28.
//

#ifndef FINALEXAM_RC_KEY_H
#define FINALEXAM_RC_KEY_H
#include "RemoteC.h"
#include "image_referee.h"
/* Key Define List */
/**
 * 英文解释
 * HOLD 长按
 * CLICK 点按
 */
#define SHOOT_NORMAL_HOLD RC_GetDatas().mouse.press_l.Now_State   //鼠标左键长按 手动发射
#define SHOOT_MODE_CLICK RC_GetDatas().key.R.Is_Click_Once    //单发连发更改
#define SHOOT_NORMAL RC_GetDatas().mouse.press_l      //鼠标左键
#define SHOOT_AUTOAIM_HOLD RC_GetDatas().mouse.press_r.Now_State  //鼠标右键长按 自瞄
#define RAMMC_REVERSE RC_GetDatas().key.Z.Now_State //长按 拨弹轮反拨
#define CHASSIS_ACC_HOLD RC_GetDatas().key.SHIFT.Now_State  //长按 底盘加速 用于飞坡
#define CHASSIS_ANGLE_CLICK RC_GetDatas().key.CONTRL.Is_Click_Once //点按 底盘角度切换 45°侧身面敌 - 0°正面面敌
#define CHASSIS_ANGLE RC_GetDatas().key.CONTRL //点按 底盘角度切换 45°侧身面敌 - 0°正面面敌
#define TURN_BACK_CLICK  RC_GetDatas().key.Q.Is_Click_Once //点按 一键转身
#define TURN_BACK  RC_GetDatas().key.Q //点按 一键转身
#define FRIC_OPEN_CLICK RC_GetDatas().key.E.Is_Click_Once //点按 打开摩擦轮
#define RAMMC_ACC_HOLD RC_GetDatas().key.F.Now_State  //长按 拨弹轮加速
#define CHASSIS_MODE_SWITCH_CLICK RC_GetDatas().key.V.Is_Click_Once    //点按 随动和小陀螺切换
//#define CHASSIS_MODE_SWITCH_CLICK RC_GetDatas().key.CONTRL.Is_Click_Once //点按 随动和小陀螺切换
#define CHASSIS_SPIN_MODE_SWITCH_CLICK RC_GetDatas().key.C.Is_Click_Once      //点按 切换小陀螺方式 随机 匀速 正弦
#define CHASSIS_SPIN_MODE_SWITCH RC_GetDatas().key.C      //点按 切换小陀螺方式 随机 匀速 正弦
#define UI_REDRAW RC_GetDatas().key.G

//#define ENERGY_MECHANISM_CLICK RC_GetDatas().key.Z.Is_Click_Once //点按 大小符切换
//#define ENERGY_MECHANISM RC_GetDatas().key.Z

#define AIM_MODE_CLICK RC_GetDatas().key.F.Is_Click_Once //点按 自瞄 打符打机器人切换
#define AIM_MODE RC_GetDatas().key.F //点按 自瞄 打符打机器人切换

#define SENTRY_MODE_CLICK RC_GetDatas().key.B.Is_Click_Once //点按 哨兵模式切换
#define SENTRY_MODE RC_GetDatas().key.B


void RC_Levers();
void portSetMove();
bool portSetFollowUp();

#endif
