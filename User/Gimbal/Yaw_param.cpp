//
// Created by hx on 2024/9/28.
//
#include "gimbalc.hpp"
#include "RemoteC.h"


#define YAW_LIMIT 90
//#define AUTOAIM_DEADFIELD 0.1f

float portSetYaw()
{
    static float portion = TIMpiece;
    static float tar_pos = 0;
    static float _tar_pos = 0;
    static float tar_offset=0;

            switch (Car.CtrlMode)
            {
                case cCar::eRC: //
//                    yaw.MotorCtrl.c_PID.setParam(70.0, 0.3, 0.0, 16384,
//                                                  28.5, 0,80,300 );
//                    yaw.MotorCtrl.c_PID.setParam(80.0, 0.3, 0.0, 16384,
//                                                 20.0, 0,80,300 );
//                    yaw.MotorCtrl.c_PID.setParam(130.0, 1, 0.0, 16384,
//                                                 30.0, 0,40,300 );
                    yaw.MotorCtrl.c_PID.setParam(200.0, 0, 0.0, 16384,
                                                 17, 0,0,16384);
//                    yaw.MotorCtrl.c_SMC.SetParam(1.6, 95, 13, 1.1, 0.5, 16350, EXPONENT, 0.2f);
                    yaw.MotorCtrl.c_SMC.SetParam(1.6, 250, 3.5, 1.1, 0.5, 16350, EXPONENT, 0.2f);
//
//                    yaw_v2.MotorCtrl.c_PID.setParam(0.4, 0.1, 0.0, 2000,
//                                                 10, 0,0.1,2000 );
                    _tar_pos -= (float) RC_GetDatas().rc.ch[0] * portion * 360.0f / 660.0f;
//                    if (fabsf(_tar_pos) >= YAW_LIMIT)//
//                        _tar_pos = YAW_LIMIT * (_tar_pos) / fabsf(_tar_pos);
                    tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                    break;
                case cCar::eKey:
                    switch (Car.ShootMode)
                    {
                        case cCar::eNormal:
//                            yaw.MotorCtrl.c_SMC.SetParam(1.6, 95, 13, 1.1, 0.5, 16350, EXPONENT, 0.2f);
//                            yaw.MotorCtrl.c_SMC.SetParam(1.6, 95, 9, 1.1, 0.5, 16350, EXPONENT, 0.2f);
                            yaw.MotorCtrl.c_SMC.SetParam(1.6, 250, 6.5, 1.1, 0.5, 16350, EXPONENT, 0.2f);
                            yaw.MotorCtrl.c_PID.setParam(200.0, 0, 0.0, 16384,
                                                         8, 0,0,16384);
                            _tar_pos -= (float) RC_GetDatas().mouse.x * mouse_sense;
//                            if (fabsf(_tar_pos) >= YAW_LIMIT)
//                                _tar_pos = YAW_LIMIT * (_tar_pos) / fabsf(_tar_pos);
                                tar_pos =_tar_pos;
//                            tar_pos = yaw.RCcrtl_filter.ckalman.Calc(_tar_pos);
                            break;
                        case cCar::eAutoaim:

//                                yaw.MotorCtrl.c_SMC.SetParam(1.6, 95, 11, 1.1, 0.1, 16350, EXPONENT, 0.2f);
//                                yaw.MotorCtrl.c_SMC.SetParam(1.6, 95, 13, 1.1, 0.5, 16350, EXPONENT, 0.2f);
                            yaw.MotorCtrl.c_SMC.SetParam(1.6, 55, 22, 1.1, 0.5, 16350, EXPONENT, 0.2f);
                                yaw.MotorCtrl.c_PID.setParam(200.0, 0, 0.0, 16384,
                                                             17, 0,0,16384);

                            if (yaw.autoaimflag)
                            {
                                if (fabs(vision_pkt.offset_yaw) >= 0.3f)
                                {
                                    _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + vision_pkt.offset_yaw - tar_offset;//
//                                    _tar_pos += vision_pkt.offset_yaw;//待测试
                                }
                                yaw.autoaimflag = false;
                            }
                            tar_pos = _tar_pos;
//                            tar_pos = yaw.autoAim_filter.cLowPass.filter(_tar_pos);
//                            usart_printf("111\r\n");
                            break;
                        case cCar::ePatrol:
                            yaw.MotorCtrl.c_SMC.SetParam(1.6, 250, 3.5, 1.1, 0.5, 16350, EXPONENT, 0.2f);
//                            _tar_pos = (float)yawSineWave.updateAndGetSineValue();///todo 待测
                            _tar_pos -= 0.05f;
                            tar_pos = _tar_pos;
                            break;
                    }
                    break;
                case cCar::eRC_Autoaim:
                    yaw.MotorCtrl.c_PID.setParam(200.0, 0, 0.0, 16384,
                                                 17, 0,0,16384);
//                    if(Car.AimMode == cCar::eRobot)
//                    {
//                        yaw.MotorCtrl.c_SMC.SetParam(1.6, 95, 11, 1.1, 0.1, 16350, EXPONENT, 0.2f);
////                        yaw.MotorCtrl.c_SMC.SetParam(1.6, 95, 13, 1.1, 0.5, 16350, EXPONENT, 0.2f);
//
//                    }
//                    else
//                    {
                    yaw.MotorCtrl.c_SMC.SetParam(1.6, 55, 22, 1.1, 0.5, 16350, EXPONENT, 0.2f);
//                    }
//                    yaw.MotorCtrl.c_SMC.SetParam(1.1,80,11,0.8,0.1,16350,POWER,0.2f);
//                    yaw.MotorCtrl.c_SMC.SetParam(Debug_Param().vel_kp, Debug_Param().vel_ki,Debug_Param().vel_kd,Debug_Param().vel_rampTargetValue,Debug_Param().vel_maxIntegral,16350,EXPONENT,0.2f);
                    if (yaw.autoaimflag)
                    {
                        if (fabs(vision_pkt.offset_yaw) >= 0.3f)
                        {
                            _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw) + vision_pkt.offset_yaw - tar_offset;// yaw.getEcd().angle-Car._p_UAVInEcd_yaw
//                            _tar_pos += vision_pkt.offset_yaw;
                        }
                         yaw.autoaimflag = false;
                    }
                    tar_pos = _tar_pos;
//                    tar_pos = yaw.autoAim_filter.cLowPass.filter(_tar_pos);
                    break;
                case cCar::eProtect: //保护
                    yaw.MotorCtrl.c_SMC.SetParam(1.6, 100, 9, 1.1, 0.5, 16350, EXPONENT, 0.2f);

                    _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Yaw);
                    break;
            }

    if ((TURN_BACK.Now_State == 1 && TURN_BACK.Last_State == 0 && TURN_BACK_CLICK) || (image_ctrl.key.Q.Now_State == 1 && image_ctrl.key.Q.Is_Click_Once  == 1))
    {
//        Car.setPCarAngleOffset(180);
//            UI_Front(LED_ON);
        tar_offset =180;
    } else if ((TURN_BACK.Now_State == 1 && TURN_BACK.Last_State == 0 && (!TURN_BACK_CLICK)) || (image_ctrl.key.Q.Now_State == 1 && image_ctrl.key.Q.Is_Click_Once  == 0))
    {
//            Car.ChassisAngle = cCar::eZero;
        tar_offset = 0;
//            UI_Front(LED_OFF);
    }

    //yaw轴数据发送
//    usart_printf("%.2f,%.2f,%.2f,%.2f\r\n",  tar_pos,yaw.getEcd().speed,yaw.getEcd().total_angle, yaw.getEcd().torque_current);//���˻��������
//    usart_printf("%.2f,%.2f,%2f,%.2f\r\n", tar_pos, IMU.cAngle(cimu::CH100_imu, cimu::Yaw), vision_pkt.offset_yaw,
//                 _tar_pos);

//    usart_printf("%.2f,%.2f,%.2f\r\n", yaw_mat.Out(), tar_pos, IMU.cAngle(cimu::CH100_imu, cimu::Yaw));
    //
//    usart_printf("%.2f,%.2f,%.2f,%.2f\r\n", tar_pos, IMU.cAngle(cimu::Wit_imu, cimu::Yaw), yaw.MotorCtrl.c_SMC.Out(),
//                 vision_pkt.offset_yaw);
//    usart_printf("%.2f,%.2f,%.2f,%.2f\r\n",IMU.cAngle(cimu::Wit_imu,cimu::Yaw),vision_pkt.offset_yaw,_tar_pos,tar_pos);
//    usart_printf("%.2f,%.2f\r\n",tar_pos,yaw.getEcd().total_angle + Car._p_UAVInEcd_yaw_offset - Car._p_UAVInEcd_yaw);
//    usart_printf("%.2f,%.2f,%.2f\r\n",tar_pos, IMU.cAngle(cimu::Wit_imu, cimu::Yaw), vision_pkt.offset_yaw);
//    usart_printf("%f\r\n",IMU.cAngle(cimu::Wit_imu,cimu::Yaw) + yawSineWave.updateAndGetSineValue());
//
    return tar_pos + tar_offset;
}
