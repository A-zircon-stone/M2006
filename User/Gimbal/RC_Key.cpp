//
// Created by hx on 2024/9/28.
//
#include "gimbalc.hpp"
#include "RemoteC.h"
#include "LEDC.h"
#include "RC_Key.h"
#include "image_referee_struct.h"

/**
 * 遥控器 获取拨杆控制信息
 */
void RC_Levers()
{
    /*************CtrlMode********************************************************/
///图传链路优先
//    if (C5_portSetProtect())
//    {
//        if (RC_GetDatas().rc.s[1] == 2 && RC_GetDatas().rc.s[0] == 3 )Car.CtrlMode = cCar::eKey;//左下右中
//        else Car.CtrlMode = cCar::eRC;
//    } else
//    {
//        if (image_ctrl.rc.Middle_Key == 0) Car.CtrlMode = cCar::eImageKey;
//        else if (image_ctrl.rc.Middle_Key == 2) Car.CtrlMode = cCar::eImageAutoaim;
//        else Car.CtrlMode = cCar::eImage;
//    }
///DT7优先
//    if (RC_GetNewData.live_time <= 0)
//    {
//        if (image_ctrl.rc.Middle_Key == 0) Car.CtrlMode = cCar::eImageKey;
//        else if (image_ctrl.rc.Middle_Key == 2) Car.CtrlMode = cCar::eImageAutoaim;
//        else Car.CtrlMode = cCar::eImage;
//    } else
//    {
        if (RC_GetDatas().rc.s[1] == 2 && RC_GetDatas().rc.s[0] == 3 )Car.CtrlMode = cCar::eKey;//左下右中
        else Car.CtrlMode = cCar::eRC;
//    }
        /*************CtrlMode********************************************************/

    /*************RC********************************************************/
    //右拨杆
    switch (Car.CtrlMode)
    {
        case cCar::eRC:
            if (RC_GetDatas().rc.s[0] == 1 )//小陀螺
            {
//                setSpin(100);
                if (RC_GetDatas().rc.s[1] == 2) setSpin(-100);//小陀螺反转
                else setSpin(100);
                Car.ChassisMode = cCar::eSpin;
                UI_gyroscope(0);
            }
            else
            {
                Car.ChassisMode = cCar::eFollow;
                UI_gyroscope(3);
            }
            if (RC_GetDatas().rc.s[0] == 2)//遥控器下的自瞄模式
                Car.CtrlMode = cCar::eRC_Autoaim;
            break;
    }

    //开启摩擦轮操作
    switch (Car.CtrlMode)
    {
        case cCar::eRC:
            if (RC_GetDatas().rc.s[1] == 1)//左拨杆上拨
            {
                Car.FricCtrlMode = cCar::eFric_OPEN;
                Car.ShootMode = cCar::eNormal;      //手动发射
            }
            if (RC_GetDatas().rc.s[1] == 3)//左拨杆置中
                Car.FricCtrlMode = cCar::eFric_SHUT;
            break;
        case cCar::eRC_Autoaim:
            switch (RC_GetDatas().rc.s[1])
            {
                case 1:     //上拨
                    Car.FricCtrlMode = cCar::eFric_OPEN;
                    Car.ShootMode = cCar::eNormal;
                    break;
                case 2:     //下拨
                    Car.FricCtrlMode = cCar::eFric_OPEN;
                    Car.ShootMode = cCar::eAutoaim;
                    break;
                case 3:     //置中
                    Car.FricCtrlMode = cCar::eFric_SHUT;
                    break;
            }
            break;
        default:
            break;
    }
    /*************RC********************************************************/

    /*************KEY********************************************************/
    if (Car.CtrlMode == cCar::eKey)
    {
        if (UI_REDRAW.Now_State)
        {
            CHASSIS_YAW.CANx_send_data[7] = 1;
        }
        else
            CHASSIS_YAW.CANx_send_data[7] = 0;
        //打开摩擦轮
        if (FRIC_OPEN_CLICK)
        {
            Car.FricCtrlMode = cCar::eFric_OPEN;
        }
        else Car.FricCtrlMode = cCar::eFric_SHUT;

        //发射 自瞄或者手动 拨弹盘加速或者不加速
        if (SHOOT_AUTOAIM_HOLD)
        {
            Car.ShootMode = cCar::eAutoaim;
//            switch (RAMMC_ACC_HOLD)
//            {
//                case 1:
//                    Car.RammcMode = cCar::eRammcAccelerate;
//                    break;
//                case 0:
//                    Car.RammcMode = cCar::eRammcNormal;
//                    break;
//            }
        }
        else Car.ShootMode = cCar::eNormal;
//        if (Car.ShootMode == cCar::eNormal)
//        {
//            if (SHOOT_NORMAL_HOLD)
//            {
//                Car.ShootMode = cCar::eNormal;
//                switch (RAMMC_ACC_HOLD)
//                {
//                    case 1:
//                        Car.RammcMode = cCar::eRammcAccelerate;
//                        break;
//                    case 0:
//                        Car.RammcMode = cCar::eRammcNormal;
//                        break;
//                }
//            }
//        }


//        if (SHOOT_MODE_CLICK)  //判定条件为上次记录状态为1，且本次为0（进行切换过）
//        {
//            Car.AmmoMode = cCar::eSingleAmmo;
////            UI_Protect(LED_ON);
//        }
//        else
//        {
//            Car.AmmoMode = cCar::eContinuousAmmo;
//            UI_Protect(LED_OFF);
//        }

        //底盘状态 45°侧向面敌
        if (CHASSIS_ANGLE.Now_State == 1 && CHASSIS_ANGLE.Last_State == 0 && CHASSIS_ANGLE_CLICK)
        {
            if(Car.ChassisAngle == cCar::eZero)
            {
                Car.ChassisAngle = cCar::eForty_five;
            }
            else if(Car.ChassisAngle == cCar::eBack)
            {
                Car.ChassisAngle = cCar::eBack_Forty_five;
            }
            Car.setPCarAngleOffset(45);
//            UI_Front(LED_ON);
        }
        else if (CHASSIS_ANGLE.Now_State == 1 && CHASSIS_ANGLE.Last_State == 0 && (!CHASSIS_ANGLE_CLICK))
        {
            Car.setPCarAngleOffset(0);
            if(Car.ChassisAngle == cCar::eForty_five)
            {
                Car.ChassisAngle = cCar::eZero;
            }
            else if(Car.ChassisAngle == cCar::eBack_Forty_five)
            {
                Car.ChassisAngle = cCar::eBack;
            }
            UI_Front(LED_OFF);
        }
        if (TURN_BACK.Now_State == 1 && TURN_BACK.Last_State == 0 && TURN_BACK_CLICK)
        {
            Car.setPturnCarAngleOffset(180);
            if(Car.ChassisAngle == cCar::eZero)
            {
                Car.ChassisAngle = cCar::eBack;
            }
            else if(Car.ChassisAngle == cCar::eForty_five)
            {
                Car.ChassisAngle = cCar::eBack_Forty_five;
            }


//            UI_Front(LED_ON);
        }
        else if (TURN_BACK.Now_State == 1 && TURN_BACK.Last_State == 0 && (!TURN_BACK_CLICK))
        {

            Car.setPturnCarAngleOffset(0);
            if(Car.ChassisAngle == cCar::eBack)
            {
                Car.ChassisAngle = cCar::eZero;
            }
            else if(Car.ChassisAngle == cCar::eBack_Forty_five)
            {
                Car.ChassisAngle = cCar::eForty_five;
            }
//            UI_Front(LED_OFF);
        }

        if(Car.ChassisAngle == cCar::eZero)
        {
            UI_Front(LED_OFF);
        }
        else if(Car.ChassisAngle == cCar::eForty_five)
        {
            static int j = 0;
            if(j >= 50)
            {
                j = 0;
                UI_Front(LED_TOGGLE);
            }
            j++;
        }
        else if(Car.ChassisAngle == cCar::eBack)//朝向指示灯
        {
            static int i = 0;
            if(i >= 20)
            {
                i = 0;
                UI_Front(LED_TOGGLE);
            }
            i++;
        }
        else{
            UI_Front(LED_ON);
        }

        //哨兵模式
        if(SENTRY_MODE_CLICK)
        {
            yaw.clearEcdRcnt();

            Car.FricCtrlMode = cCar::eFric_OPEN;

            if (vision_pkt.offset_yaw ==0 && vision_pkt.offset_pitch ==0)
            {
                Car.ShootMode = cCar::ePatrol;

            } else
            {
                Car.ShootMode = cCar::eAutoaim;
            }
        }
//        else if (!SENTRY_MODE_CLICK)
//        {
//            Car.ShootMode = cCar::eNormal;
//        }

        //切换小陀螺模式
        if (Car.ChassisMode == cCar::eSpin)
        {
            static int16_t spin_speed;  //旋转速度
            static float spin_sin;    //sin时间参数
            static int64_t random_record;    //sin时间参数
            static uint8_t spin_mode_count = 0;
            static cCar::eSpinMode spin_mode[3] = {cCar::eUniform,//匀速
                                                   cCar::eRandom,//随机速度
                                                   cCar::eSin};//正弦
            if (CHASSIS_SPIN_MODE_SWITCH.Last_State != CHASSIS_SPIN_MODE_SWITCH.Now_State)
            {
                spin_mode_count++;
                if (spin_mode_count >= 3) spin_mode_count = 0;
            }
            Car.SpinMode = spin_mode[spin_mode_count];
            switch (Car.SpinMode)
            {
                case cCar::eUniform:    //匀速
                    spin_speed = 100;
                    UI_gyroscope(0);
                    break;
                case cCar::eRandom:
                    random_record++;
                    if (random_record >= 150)
                    {
                        random_record = 0;
                        spin_speed = rand() % 41 + 60;
                    }
                    UI_gyroscope(1);
                    break;
                case cCar::eSin:
                    spin_sin += 0.01;
                    spin_speed = 10 * sin(spin_sin) + 90;
                    UI_gyroscope(2);
                    break;
            }
            setSpin(spin_speed);
        }
        else UI_gyroscope(3);

    }
    /*************KEY********************************************************/


///自瞄在线检测
//    Car.is_PC_alive--;
//    if (Car.is_PC_alive<=0){
//        Car.is_PC_alive=-1;
//        UI_PC(LED_OFF);
//    }
//    else
//        UI_PC(LED_ON);

    if (vision_pkt.offset_yaw ==0 && vision_pkt.offset_pitch ==0)
        UI_PC(LED_OFF);
    else UI_PC(LED_ON);

//    if (ENERGY_MECHANISM.Now_State == 1 && ENERGY_MECHANISM.Last_State == 0 && ENERGY_MECHANISM_CLICK)
//    {
//        CHASSIS_YAW.CANx_send_data[3] = 1;
//        Car.EnergyMode = cCar::eBigFu;
//    }
//    else if (ENERGY_MECHANISM.Now_State == 1 && ENERGY_MECHANISM.Last_State == 0 && (!ENERGY_MECHANISM_CLICK))
//    {
//        CHASSIS_YAW.CANx_send_data[3] = 0;
//        Car.EnergyMode = cCar::eSmallFu;
//    }

//    if (AIM_MODE.Now_State == 1 && AIM_MODE.Last_State == 0 && AIM_MODE_CLICK)
//    {
//        CHASSIS_YAW.CANx_send_data[4] = 1;
//
//        Car.AimMode = cCar::eEnergy;
//    }
//    else if (AIM_MODE.Now_State == 1 && AIM_MODE.Last_State == 0 && (!AIM_MODE_CLICK))
//    {
//        CHASSIS_YAW.CANx_send_data[4] = 0;
//
//        Car.AimMode = cCar::eRobot;
//    }
//
//    if (CHASSIS_POWER.Now_State == 1 && CHASSIS_POWER.Last_State == 0 && CHASSIS_POWER_CLICK)
//    {
//        CHASSIS_YAW.CANx_send_data[5] = 1;
//
//        Car.ChassisPower = cCar::eWithOutCtrl;
//    }
//    else if (CHASSIS_POWER.Now_State == 1 && CHASSIS_POWER.Last_State == 0 && (!CHASSIS_POWER_CLICK))
//    {
//        CHASSIS_YAW.CANx_send_data[5] = 0;
//
//        Car.ChassisPower = cCar::ePowerCtrl;
//    }
//    usart_printf("%d\r\n", CHASSIS_YAW.CANx_send_data[5]);


}




/**
 * 控制移动
 */
void portSetMove()
{
    float vx = 0;
    float vy = 0;
    switch (Car.CtrlMode)
    {

        case cCar::eRC:
            vx = RC_GetDatas().rc.ch[2] * 200.0f / 660.0f;
            vy = RC_GetDatas().rc.ch[3] * 200.0f / 660.0f;
            break;
        case cCar::eRC_Autoaim:
            vx = RC_GetDatas().rc.ch[2] * 200.0f / 660.0f;
            vy = RC_GetDatas().rc.ch[3] * 200.0f / 660.0f;
            break;
        case cCar::eKey:    //WASD
            if (CHASSIS_ACC_HOLD)
            {
                if (RC_GetDatas().key.D)
                    vx = RC_GetDatas().key.D * 200.0f;
                else if (RC_GetDatas().key.A)
                    vx = -RC_GetDatas().key.A * 200.0f;
                if (RC_GetDatas().key.S)
                    vy = -RC_GetDatas().key.S * 200.0f;
                else if (RC_GetDatas().key.W)
                    vy = RC_GetDatas().key.W * 200.0f;
            }
            else
            {
                if (RC_GetDatas().key.D)
                    vx = RC_GetDatas().key.D * 100.0f;
                else if (RC_GetDatas().key.A)
                    vx = -RC_GetDatas().key.A * 100.0f;
                if (RC_GetDatas().key.S)
                    vy = -RC_GetDatas().key.S * 100.0f;
                else if (RC_GetDatas().key.W)
                    vy = RC_GetDatas().key.W * 100.0f;
            }
            break;
    }
    CHASSIS_VEL.CANx_send_data[0] = (int16_t) vx >> 8;
    CHASSIS_VEL.CANx_send_data[1] = (int16_t) vx & 0xff;
    CHASSIS_VEL.CANx_send_data[2] = (int16_t) vy >> 8;
    CHASSIS_VEL.CANx_send_data[3] = (int16_t) vy & 0xff;
}

/**
 * 设置随动模式
 * @return 是否随动
 */
bool portSetFollowUp()      //随动
{
    static uint8_t clear_flag = 1;
    bool mode_judge;
    switch (Car.CtrlMode)
    {

        case cCar::eRC:
            mode_judge = (RC_GetDatas().rc.s[0] == 3);//右拨杆置中
            break;
        case cCar::eRC_Autoaim:
            mode_judge = (RC_GetDatas().rc.s[0] == 2);
            break; //
        case cCar::eKey:    //切换陀螺和随动模式
            mode_judge = (CHASSIS_MODE_SWITCH_CLICK != 1 && SENTRY_MODE_CLICK !=1);
            break;
    }
    if (mode_judge)
    {
        Car.ChassisMode = cCar::eFollow;
        if (clear_flag)
        {
            yaw.clearEcdRcnt();
        }
        clear_flag = 0;
        return true;
    }
    else
    {
        Car.ChassisMode = cCar::eSpin;
        clear_flag = 1;
        yaw.clearEcdRcnt();
        return false;
    }
}

