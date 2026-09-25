//
// Created by Andrew on 2024/8/27.
//
#ifndef FINALEXAM_PIDC_H
#define FINALEXAM_PIDC_H

#include "stm32f4xx_hal.h"
#include <cmath>

#define PID_DEFAULT_PRECISION 0.1f         //控制精度，当目标速度与实际速度小于此值时，认为没有误差，使PID更稳定
#define PID_DEFAULT_ERRALL_MAX 500         //控制ERR_ALL最大值，否则ERR_ALL最大值过大，会使PID反应慢，不稳定，积分限幅
#define PID_DEFAULT_OUTPUT_MAX 3000     //输出限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX 3000 //输出微分限幅
#define PID_DEFAULT_ERRALL_MAX_POS 100 //输出积分限幅
#define PID_DEFAULT_OUTPUT_STEP_MAX_POS 500 //输出微分限幅
#define PID_DEFAULT_VSC_Max 100000                 //变速积分上限
#define PID_DEFAULT_VSC_Min 10000                 //变速积分下限

typedef enum
{
    eClassic,
    eVSC,   //变速积分
} PID_CalcType_e;

typedef enum
{
    Normal_e = 0,            ////PID工作在正常状态
    Ramp_e = 1,                // PID工作在斜坡函数状态
    Ramping_Add = 2,        //斜坡还没完成，并且是加
    Ramping_Reduce = 3,        //斜坡还没完成，并且是减

    RampDefault_Step = 2, //斜坡函数步幅
    RampDefault_Time = 1,    //计数多少次才把count加上1个步幅
} PID_RampState_e;

typedef struct
{
    float Kp1;
    float Ki1;
    float Kd1;
    float forward;
    float PID_Err_now;
    float PID_Err_last;
    float PID_Err_lastlast;
    float PID_Err_all;
    float PID_Out;
    float PID_lastout;
    float PID_Target;
    float PID_lastTarget;
    float PID_Input;
    float PID_LastInput;
    /*  变速积分  */
    float PID_VSCportion;       //Variable_speed_credits 变速积分系数
    float PID_VSCalpha;       //Variable_speed_credits 变速积分系数
    float PID_VSCmax;           //变速积分比例范围上限
    float PID_VSCmin;           //变速积分比例范围下限

    uint8_t State_RampOrNormal;
    float RampTarget;
    uint8_t RampCountTime;
    uint8_t RampTargetTime; //计数多少次才把count加上1个步幅
    float RampTargetStep; //斜坡函数步幅
    uint8_t PID_WorkType;  // PID工作在位置式还是增量式
    uint8_t Connected;
    float PID_Precision; // PID最小精度
    float PID_ErrAllMax; // PID积分限幅
    float PID_OutMax;     // PID输出限幅
    float PID_OutStep;     // PID输出步幅限制
    PID_CalcType_e CalcType;//选择pid的计算方式
} PID_t;

void PID_Update(PID_t *WhichPID, float NowInput);

float PID_GetPositionPID(PID_t *WhichPID);

void PID_SpeedParamInit(PID_t *WhichPID);

void PID_PosParamInit(PID_t *WhichPID);

void PID_SetTargetWithRamp(PID_t *WhichPID, float Tartget);

void PID_Clear(PID_t *WhichPID);

class cPID
{
private:
    PID_t SpdParam;
    PID_t PosParam;
public:
    void Init();
    void setParam(float pos_kp, float pos_ki, float pos_kd, float pos_outmax);
    void setParam(float spd_kp, float spd_ki, float spd_kd, float spd_outmax,
                  float pos_kp, float pos_ki, float pos_kd, float pos_outmax);
    void setParam(float spd_kp, float spd_ki, float spd_kd, float spd_outmax,
                  float pos_kp, float pos_ki, float pos_kd, float pos_outmax, float pos_forward);
    void setParam(float spd_kp, float spd_ki, float spd_kd, float spd_tar, float spd_outmax);//速度环调试
    void setParam(float spd_kp, float spd_ki, float spd_kd, float spd_outmax,
                  float pos_kp, float pos_ki, float pos_kd, float pos_outmax, float pos_tar, float pos_forward);//位置环调试
    /** POS **/
    void setPosTar(float pos_tar);
    void setPosTar(float pos_tar, float step);      //斜坡 step步进
    float posLoop(float pos_input);
    /*
     * 串级PID位置环
     * 参数：反馈的当前位置、速度
     * */
    void posLoop(float pos_input, float spd_input);
    /** SPD **/
    void setSpdTar(float spd_tar);
    void setSpdTar(float spd_tar, float step);
    void spdLoop(float spd_input);

    float getSpdTar();

    /** PID_Clear**/
    void SpdPIDClear();
    void PosPIDClear();
    /** PID_Set**/
    void PosPIDOutSet(float out);
    void SpdPIDOutSet(float out);


    const PID_t &getSpdParam() const;
    const PID_t &getPosParam() const;
    [[nodiscard]] float pidOut() const;         //PID算法输出
};

#endif //FINALEXAM_PIDC_H
