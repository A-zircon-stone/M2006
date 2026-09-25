//
// Created by hx on 2025/1/11.
//

#ifndef INFANTRY_GIMBAL_SLIDING_H
#define INFANTRY_GIMBAL_SLIDING_H
#include <cmath>
#include "DebugC.h"

#define SAMPLE_PERIOD 0.002
#define V_EORROR_INTEGRAL_MAX 2000
#define P_EORROR_INTEGRAL_MAX 2000

typedef enum {
    EXPONENT,
    POWER,
    TFSMC,
    VOLSMC,
    EISMC
} Rmode;

typedef struct {

    float tar_now;
    float tar_last;
    float tar_differential;
    float tar_differential_last;
    float tar_differential_second;

    float pos_get;
    float vol_get;

    float p_error;
    float v_error;

    float p_error_integral;
    float v_error_integral;
//    float v_error_integral_max;

    float pos_error_eps;   //误差精度
    float vol_error_eps;   //误差精度
    float error_last;
}RError;

typedef struct {
    float J;
    float K;
    float c;

    float c1;
    float c2; //EIsmc参数

    float p;    //正奇数 p>q
    float q;    //正奇数
    float beta; //正数
    float epsilon; //ε噪声上限
}SlidingParam;

typedef struct {
    float u;
    float s;

    SlidingParam param;
    SlidingParam param_last;

    RError error;
    float u_max;
    Rmode flag; //符号和饱和切换，未用到
    float limit; //饱和函数上下限
}Sliding;





class cSMC
{
public:
    void Init();

    void SetParam(float J, float K, float c, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); //滑膜参数设定 EXPONENT,POWER,VOLSMC 参数
    void SetParam(float J, float K, float p, float q, float beta, float epsilon, float limit, float u_max, Rmode flag, float pos_esp);//滑膜参数设定 TFSMC 参数
    void SetParam(float J, float K, float c1, float c2, float epsilon, float limit, float u_max, Rmode flag, float pos_esp); ///EISMC 参数设定

    void ErrorUpdate(float target, float pos_now, float vol_now); //滑膜位置误差更新
    void ErrorUpdate(float target,float vol_now); //滑模速度误差更新
    void Clear();
    void Integval_Clear();

    float SmcCalculate(); //滑模控制器计算函数
    float Out();
    void SetOut(float out);
    const Sliding &getSmc() const; //参量计算

private:


    Sliding smc;
    void OutContinuation(); //输出连续化
    float Signal(float s); //符号函数
    float Sat(float s); //饱和函数
};
#endif //INFANTRY_GIMBAL_SLIDING_H
