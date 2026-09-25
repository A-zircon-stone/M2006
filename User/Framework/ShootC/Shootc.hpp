

#ifndef KOSANN_INFANTRY_GIMBAL_SHOOTC_HPP
#define KOSANN_INFANTRY_GIMBAL_SHOOTC_HPP

#include "Framework/referee/refee.hpp"
#include "MotorC.hpp"
#include "../../MotorC.hpp"
#include <cmath>

#define Reverse_Time_Max 20  //反拨时长
#define Ammunition_Total 400 //总弹量
#define single_ammo_angle 45    //鼠标单发，检测转一个弹需要的角度
#define reverse_time_max 400     //反转最大时间检测次数
#define return_angle 45        //反转角度
#define stalled_detect_time 100 //堵转检测时间
#define shoot_detect_time 4  //发弹检测时间
#define SHOOT_INTERVAL_TIME 10 //拨弹盘目标值间隔更新时间
#define REFER_DELAY_TIME 1000 //裁判系统更新超时等待时间

class Shootc {
public:
    typedef struct {
        float heat_limit = 50; //枪管热量限制
        float cool_rate; //枪管冷却速度

        float heat_now_referee; //裁判系统枪管当前热量
        float heat_last_referee; //裁判系统枪管上状态热量
        int ammo_state; //ammo口输出信息
        int ammo_state_last; //ammo口上一次输出信息

        float heat_now; //计算枪管当前热量

        bool fric_state;

        float shoot_spd_last; //上次射速 用于判定是否发弹 暂定，可能不需要
        float shoot_spd; //当前射速
        uint16_t shoot_spd_limit; //弹速限制

        int ammunition; //已发弹量
    } eShootData_t;


    typedef enum
    {
        eFric_ON,        //控制开启且摩擦轮速度达到射击要求
        eFric_OFF        //结束控制
    } eFricState;//摩擦轮状态表


    typedef enum
    {
        eRammc_Permitted = 1,         //允许拨弹轮拨弹
        eRammc_Down = 2,
        eRammc_Cool = 3,
        eRammc_Stopped = 4         //阻止拨弹轮拨弹
    }eRammcState; //拨弹轮控制状态表，用于热量控制

    typedef struct
    {
        bool fric_check;        //摩擦轮掉速标志(0为正常，1为掉速)
        bool rammc_single_permit;       //允许单发发射子弹 false允许，true阻止

        float rammc_speed; //拨弹轮转速

        bool is_shot;        //是否发射
        uint16_t shooted_num;     //已发送发射弹数
        bool is_stalled;        //堵转检测
        uint16_t delay_time;
    }eShootCheck;


private:
    cMotor *fricR, *fricL;
    cMotor *rammc;
    eShootData_t ShootData;
    eFricState FricState;
    eRammcState RammcState;
    eShootCheck ShootCheck;


public:
    Shootc(cMotor *fricR_t, cMotor *fricL_t, cMotor *rammc_t);

    void ShootDataUpdate(sReferee_Msg msg); //射击数据更新，主要是将裁判系统采集进来
    void SetShootLoop(cCar car_t);  //暂定射击循环 外部调用接口
    void SetRammcShoot(cCar car_t); //射击拨弹轮控制

    void AmmunitionAdd();
    void AmmunitionClean();


private:
    void StallCheck(cCar car_t); //堵转检测
    void SetFricShoot(cCar car_t); //射击摩擦轮控制
    void HeatProtect(); //热量检测控制
    void FricCheck(); //摩擦轮掉速检测 拨弹检测部分
    void RammcCheck(); //拨弹轮电流超限检测 拨弹检测部分
    void FricRevive();//ammo口重新上电自我保护


};



#endif //KOSANN_INFANTRY_GIMBAL_SHOOTC_HPP
