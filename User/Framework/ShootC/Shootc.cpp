
#include "Shootc.hpp"
#include "RC_Key.h"
#include "LEDC.h"
#include "gimbalc.hpp"



Shootc::Shootc(cMotor *fricR_t, cMotor *fricL_t, cMotor *rammc_t) : fricR(fricR_t), fricL(fricL_t), rammc(rammc_t)
{
    FricState  = eFric_OFF;
};

void Shootc::ShootDataUpdate(sReferee_Msg msg) //裁判系统数据更新
{
    ShootData.heat_last_referee = ShootData.heat_now_referee;//寄存上一状态裁判系统热量用于计算热量的校正

    ShootData.shoot_spd_last = ShootData.shoot_spd;

    ShootData.heat_limit = (float) msg.heat_limit;
    ShootData.cool_rate = (float) msg.cool_rate;
//    ShootData.cool_rate = 8;

    ShootData.heat_now_referee = (float) msg.heat_now;
    ShootData.ammo_state = msg.ammo_state;

    ShootData.shoot_spd = (float) msg.shoot_spd;
    ShootData.shoot_spd_limit = msg.shoot_spd_limit;
//    usart_printf("%.2f,%.2f,%.2f,%d\r\n", ShootData.heat_limit,ShootData.cool_rate,ShootData.heat_now_referee,ShootData.ammo_state);
}

void Shootc::SetFricShoot(cCar car_t)
{
    static int revive_time = 0;
    static int shoot_permit_time = 0;
    if (ShootData.ammo_state == 0)
    {
        FricState = eFric_OFF;
        fricL->MotorCtrl.c_PID.SpdPIDClear();
        fricR->MotorCtrl.c_PID.SpdPIDClear();
        revive_time = 0;
        static int j = 0;
        if(j>=30)
        {
            UI_Fric(LED_TOGGLE);
            j = 0;
        }
        j++;
    }
    else
    {
        revive_time++;
    }
    if(revive_time >= 750)
    {
        if ((fricR->getEcd().speed <= 5900 || fricL->getEcd().speed >= -5900) || (fricR->getEcd().speed >= 6300 || fricL->getEcd().speed <= -6300)) ///摩擦轮速度不满足
        {
            FricState = eFric_OFF;
            shoot_permit_time = 0;
            if(fricR->getEcd().speed >= 30 || fricL->getEcd().speed <= -30)
            {
                static int i = 0;
                if(i>=15)
                {
                    UI_Fric(LED_TOGGLE);
                    i = 0;
                }
                i++;
            }
            else
            {
                UI_Fric(LED_OFF);
            }
        }
        if (car_t.FricCtrlMode == cCar::eFric_SHUT)  // 摩擦轮关闭，设置摩擦轮速度为0
        {
            fricL->MotorCtrl.c_PID.setSpdTar(0);
            fricR->MotorCtrl.c_PID.setSpdTar(0);
            FricState = eFric_OFF;
            UI_Fric(LED_OFF);
        }
        else if (car_t.FricCtrlMode == cCar::eFric_OPEN)  //摩擦轮开启
        {
            fricL->MotorCtrl.c_PID.setSpdTar(-6200.0f);
            fricR->MotorCtrl.c_PID.setSpdTar(6200.0f);
            if (fricR->getEcd().speed > 5900 && fricL->getEcd().speed < -5900 && fricR->getEcd().speed <= 6300 && fricL->getEcd().speed >= -6300) /// 摩擦轮速度满足发弹要求
            {
                shoot_permit_time++;
            }
            if(shoot_permit_time >= 20)
            {
                FricState = eFric_ON;
                UI_Fric(LED_ON);
            }
        }
    }
//    usart_printf("%d,%.1f,%.1f\r\n", 6700, -fricL->getEcd().speed, fricR->getEcd().speed);
}

void Shootc::StallCheck(cCar car_t)
{
    static int32_t stuck_time = 0;
    static int32_t reverse_time = 0;
    int16_t Current = rammc->getEcd().torque_current;
    static float pos_target;

    if (abs(Current) > 7500) //卡弹堵转
    {
        stuck_time++;
//        usart_printf("%d,%d,%f\r\n", rammc->getEcd().torque_current,stuck_time,rammc->MotorCtrl.c_PID.getPosParam().PID_Target);
        if (stuck_time >= stalled_detect_time)
        {
            ShootCheck.is_stalled = true;
            stuck_time = 0;
            pos_target = rammc->getEcd().total_angle - return_angle;
        }
    }
    if (ShootCheck.is_stalled)
    {
        if (rammc->getEcd().total_angle >= pos_target)
        {
            reverse_time++;
            if (reverse_time <= reverse_time_max)
//            rammc->MotorCtrl.c_PID.PosPIDClear();
//            rammc->MotorCtrl.c_PID.setPosTar(pos_target);
            {
                rammc->MotorCtrl.c_SMC.Integval_Clear();
                rammc->MotorCtrl.c_SMC.ErrorUpdate(pos_target, rammc->getEcd().total_angle, rammc->getEcd().speed);
            }
            else
            {      //else:堵转后反转时间超时 说明机械卡弹 此时已无法反转 //todo:解决方案：发送信息给UI说明情况？
//                rammc->MotorCtrl.c_PID.SpdPIDClear();
//                rammc-> MotorCtrl.c_PID.setSpdTar(0);
//                usart_printf("mechanically_stalled\r\n");
                ShootCheck.is_stalled = false;
                ShootCheck.rammc_single_permit = false;
                reverse_time = 0;
            }
        }
        else
        {
            ShootCheck.is_stalled = false;
            ShootCheck.rammc_single_permit = false;
            reverse_time = 0;
        }
    }
//    usart_printf("%.2f,%.2f,%.2f\r\n", pos_target, rammc->getEcd().total_angle,rammc->getEcd().torque_current);
}


void Shootc::SetRammcShoot(cCar car_t)
{
    static float tar_pos = rammc->getEcd().total_angle;
    static bool tar_update_flag = 1;
    static int32_t hold_time = 0;
    static int32_t interval_time = 0;

//    rammc->MotorCtrl.c_PID.setParam(150,0,0,10000,
//                                    10,0,0,500);

    if (car_t.CtrlMode == cCar::eProtect)
    {
        tar_pos = rammc->getEcd().total_angle;
    }

    if ((FricState == eFric_OFF) || (RammcState == eRammc_Stopped)) //摩擦轮未准备好 或 热量不允许发射
//    if (FricState == eFric_OFF)///热量控制加入前暂用
    {
        if (!tar_update_flag)
        {
            if (hold_time >= 25)
            {
                tar_pos = rammc->getEcd().total_angle;
                tar_update_flag = 1;
            }
            hold_time++;
        }
//        rammc->MotorCtrl.c_PID.SpdPIDClear(); //清除PID计算过程中的误差、积分、微分等数据量
//        rammc->MotorCtrl.c_PID.setSpdTar(0.0f);
//        tar_pos = rammc->getEcd().total_angle;
    }
    if ((FricState == eFric_ON) && (RammcState == eRammc_Permitted)) //摩擦轮已准备 且 热量允许继续发射
//    if (FricState == eFric_ON)///热量控制加入前暂用
    {
        tar_update_flag = 0;
        hold_time = 0;
        switch (car_t.CtrlMode)
        {
            case cCar::eRC:
                if (RC_GetDatas().rc.ch[4] < -100) // 遥控轮盘控制拨弹轮
                {

                    if (!ShootCheck. rammc_single_permit && !ShootCheck.is_stalled)
                    {
                        tar_pos = rammc->getEcd().total_angle + single_ammo_angle;
//                        AmmunitionAdd();
                        ShootCheck.rammc_single_permit = true;
//                        tar_pos -= (float ) RC_GetDatas().rc.ch[4] * portion * 360.0f / 660.0f;
                    }
                    interval_time++;
                }
//
                break;
            case cCar::eKey:
                switch (car_t.ShootMode)
                {
                    case cCar::eNormal:
                        if (SHOOT_NORMAL_HOLD)
                        {
                            if (!ShootCheck. rammc_single_permit && !ShootCheck.is_stalled)
                            {
                                tar_pos = rammc->getEcd().total_angle + single_ammo_angle;
                                ShootCheck.rammc_single_permit = true;
                            }
                            interval_time++;
                        }
                    break;
                    case cCar::eAutoaim:
                        if (vision_pkt.suggest_fire)
                        {
                            if (!ShootCheck. rammc_single_permit && !ShootCheck.is_stalled)
                            {
                                tar_pos = rammc->getEcd().total_angle + single_ammo_angle;
                                ShootCheck.rammc_single_permit = true;
                            }
                            interval_time++;
                        }
                        break;
                }
                break;
            case cCar::eRC_Autoaim:
                switch (car_t.ShootMode)
                {
                    case cCar::eAutoaim:
                        if (vision_pkt.suggest_fire)
                        {
                            if (!ShootCheck. rammc_single_permit && !ShootCheck.is_stalled)
                            {
                                tar_pos = rammc->getEcd().total_angle + single_ammo_angle;
                                ShootCheck.rammc_single_permit = true;
                            }
                            interval_time++;
                        }
                        break;
                    case cCar::eNormal:
                        if (RC_GetDatas().rc.ch[4] < -100)
                        {
                            if (!ShootCheck. rammc_single_permit && !ShootCheck.is_stalled)
                            {
                                tar_pos = rammc->getEcd().total_angle + single_ammo_angle;
                                ShootCheck.rammc_single_permit = true;
                            }
                            interval_time++;
                        }
                        break;
                }
                break;
        }

        if (interval_time >= 70 && rammc->getEcd().total_angle >= tar_pos)
        {
            static int pause_time = 0;
            if(pause_time >= SHOOT_INTERVAL_TIME)
            {
                ShootCheck.rammc_single_permit = false;
                pause_time = 0;
                interval_time = 0;
//            rammc->MotorCtrl.c_SMC.Integval_Clear();
            }
            pause_time++;
        }
        FricCheck(); //摩擦轮掉速单发检测
    }

    rammc->MotorCtrl.c_SMC.ErrorUpdate(tar_pos,rammc->getEcd().total_angle,rammc->getEcd().speed);
//    usart_printf("%f,%f,%d\r\n",rammc->getEcd().total_angle,tar_pos,ShootCheck.rammc_single_permit);
//    rammc->MotorCtrl.c_PID.setPosTar(tar_pos);
    StallCheck(car_t); // 卡弹检测
//    usart_printf("%d\r\n", vision_pkt.suggest_fire);
//    usart_printf("%d,%d\r\n",vision_pkt.suggest_fire,ShootCheck.shooted_num);
}

void Shootc::FricCheck()
{
    static int32_t shoot_time = 0;
//    usart_printf("%d,%.2f,%.2f\r\n",ShootCheck.shooted_num,fricL->getEcd().speed,fricR->getEcd().speed);

    if (fricL->getEcd().speed >= -6000 && fricL->getEcd().speed <= -5900)
    {
        shoot_time++;
//        usart_printf("%d\r\n",shoot_time);
        if (shoot_time >= shoot_detect_time)
        {
            shoot_time = 0;
           ShootCheck.is_shot  = true;
            ShootCheck.shooted_num++;
            ShootData.heat_now += 10.0f;
        }
    }
}

void Shootc::AmmunitionAdd()
{
    ShootData.ammunition++;
    ShootData.heat_now += 10;
//    usart_printf("%d,%.2f\r\n",ShootData.ammunition,ShootData.heat_now);
}
void Shootc::HeatProtect()
{
    static int16_t refereetimer = 0; //裁判系统更新时间
    static float heat_limit = 0.0f;
//    float heat_warn = floor(ShootData.heat_limit * 0.6);
    float heat_rest = ShootData.heat_limit - ShootData.heat_now;
//    static float heat_satu = 50.0f;
//    static float heat_thrsh = 20.0f;
    static int32_t heat_protect_time = 300;

//    usart_printf("%d\r\n", ShootData.ammunition);
//    usart_printf("%.2f,%.2f,%d,%.2f,%.2f\r\n", ShootData.heat_now,ShootData.heat_now_referee,RammcState,ShootData.shoot_spd,ShootData.heat_limit);
//    usart_printf("%f,%f,%f,%f,%f\r\n",ShootData.heat_now,heat_rest,heat_warn,heat_satu,heat_thrsh);
    if (ShootData.heat_now_referee != ShootData.heat_last_referee)
    {
        ShootData.heat_now = ShootData.heat_now_referee + 20.0f;//留10热量，即留出1发的裕量
    }

    if (heat_rest <= 40)
    {
        heat_protect_time = 0;
        RammcState = eRammc_Stopped;
    }
    if (RammcState == eRammc_Stopped) heat_protect_time++;
    if (heat_protect_time >= 300)
    {
        RammcState = eRammc_Permitted;
    }
//    usart_printf("%f,%f,%f,%f\r\n",ShootData.heat_now,ShootData.heat_now_referee,ShootData.heat_limit,heat_rest);
//    usart_printf("%d\r\n",RammcState);
/*******************冷却缩减部分*****************************/
    ShootData.heat_now = ShootData.heat_now - ShootData.cool_rate / 1000.0f;  //周期是1ms

    if (ShootData.heat_now <= 0)
    {
        ShootData.heat_now = 0;
//        ShootData.ammunition = 0;
    }
/*******************冷却缩减部分*****************************/
}

void Shootc::SetShootLoop(cCar car_t)
{
    ShootDataUpdate(refereeMsg);
    HeatProtect();
    SetFricShoot(car_t);
    SetRammcShoot(car_t);
//    usart_printf("%.2f, %d, %.2f\r\n", ShootData.heat_limit, refereeMsg.cool_rate,refereeMsg.shoot_spd);
//    usart_printf("%f\r\n",rammc->getEcd().torque_current);
//    usart_printf("%f,%d\r\n",rammc->MotorCtrl.c_PID.getSpdTar(),RammcState);
}
