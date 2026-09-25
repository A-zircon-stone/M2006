//
// Created by Andrew on 2024/8/27.
//
#ifndef FINALEXAM_MOTORC_H
#define FINALEXAM_MOTORC_H

#include "framework_headfile.hpp"
#include "MCU_headfile.hpp"
#include "core.h"
//todo:讲解Motor对象的基本组成
class cMotor {
public:
    typedef enum {
        GM_6020_volt,   //电压控制 转矩电压
        GM_6020_cur,    //电机电流控制
        M_3508,
        M_2006,
        M_3508_p19,     //19:1
        M_2006_p36,      //36:1
        MF_9025
    } eMotorType;   //电机表
    enum eErrorType {
        ERROR_MOTOR,
        eCanSend_Error
    };      //错误类型
    typedef enum {
        eExternal,  //非控制算法表的算法的额外的控制量输入
        ePid,
//        eAdrc
    } eController;  //控制算法
private:
    uint32_t _can_id;
    eMotorType _motor_type;
    eController _controller_type;
    float _ratio;//减速比
    CAN_HandleTypeDef *_usehcan;
    Motor_measure_t _can_read;//获取数据,供访问
    Motor_measure_f _ecd;   //编码器数据
    uint8_t tx_channel;
    uint32_t ID;
public:
    cMotor();///todo//待删
    cMotor(uint32_t can_id_t, CAN_HandleTypeDef *hcan_t, int motor_type); //将canInit函数功能移植到构造函数中,便于成员初始化
    static void errorHandle(eErrorType error_type);

    void canInit(uint32_t can_id_t, CAN_HandleTypeDef *hcan, int motor_type);

    void canSend(eController controller_type, int16_t current);//external

    void canSend(eController controller_type);//cansend(ePID)

    void canRead(uint32_t rx_std_id, uint8_t rx_buffer[8]);

    Motor_measure_f &getEcd();

    void clearEcdRcnt();

    cFilter autoAim_filter; //自瞄滤波器
    cFilter RCcrtl_filter;  //遥控滤波器
    cFilter Filter;  //遥控滤波器
    cFilter canRv_filter; //CAN接收滤波器
    algorithm MotorCtrl;    //电机控制算法
    void protect();

    bool autoaimflag;
};

class cCar {
private:
    float _pCarInEcd;//机器人yaw轴正前方
    float _pCarPitchCentre; //机器人pitch水平机械角度
    float _pCar_Ecd_Offset;

public:
    float getPCarEcdOffset();
    float getPCarPitchCentre();

private:
    float _pCar_Angle_offset;
    float _pCar_turnAngle_offset;

public:
    float getPCarAngleOffset();
    float getPCarAngleOffsetturn();

public:
    void setPCarAngleOffset(float p_car_angle_offset);
    void setPturnCarAngleOffset(float p_car_angle_offset);

//机器人正向角度补偿
public:
    float _p_UAVInEcd_yaw;//无人机云台yaw轴正前方
    float _p_UAVInEcd_yaw_offset;//无人机自检正前方偏移值
    float _p_UAVInEcd_pitch;//无人机云台pitch轴正前方

public:

    typedef enum {
        ONMI,
        MECANUM,
        UAV
    } eCarType;//机器人表
    typedef enum {
        eRC,
        eRC_Autoaim,
        eKey,
        eProtect
    } eCtrlMode;//控制方式
    typedef enum {
        eNormal,
        eAutoaim,
        ePatrol
    } eShootMode;//射击方式

    typedef enum {
        eFric_OPEN,      //手动控制开启
        eFric_SHUT          //关闭摩擦轮
    } eFricCtrlMode;        //摩擦轮控制模式

    typedef enum {
        eFollow,    //随动
        eSpin       //小陀螺
    } eChassisMode;      //底盘运动模式

    typedef enum {
        eUniform,   //匀速
        eRandom,    //随机速度
        eSin        //正弦速度
    } eSpinMode;     //小陀螺模式

    typedef enum {
        eSingleAmmo,            //单发
        eContinuousAmmo         //连发
    } eAmmoMode;     //子弹射击模式

    typedef enum {
        eForty_five,    //45°侧身
        eZero ,         //正前方
        eBack,          //掉头
        eBack_Forty_five //掉头后的45°
    } eChassisAngle;     //底盘角度

    typedef enum {
        eRammcNormal,         //拨弹轮常速
        eRammcAccelerate    //拨弹轮加速
    } eRammcMode;       //拨弹轮状态

    typedef enum{
        eSmallFu,
        eBigFu
    }eEnergyMode;

    typedef enum{
        eRobot,
        eEnergy
    }eAimMode;

    typedef enum{
        ePowerCtrl,
        eWithOutCtrl
    }eChassisPower;

    eCarType CarType;
    eCtrlMode CtrlMode;
    eShootMode ShootMode;
    eChassisMode ChassisMode;
    eSpinMode SpinMode;
    eAmmoMode AmmoMode;
    eChassisAngle ChassisAngle;
    eRammcMode RammcMode;
    eFricCtrlMode FricCtrlMode;
    eEnergyMode EnergyMode;
    eAimMode AimMode;
    eChassisPower ChassisPower;


    explicit cCar(eCarType car_type);

    [[nodiscard]] float getPCarInEcd() const;

    void setPCarEcdOffset(float p_car_in_ecd);

    bool is_protect;    //保护模式变量
    int32_t is_PC_alive;
};

#endif //FINALEXAM_MOTORC_H
