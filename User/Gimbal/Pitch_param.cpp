//
// Created by hx on 2025/1/17.
//

#include "gimbalc.hpp"

/**
 * 重力补偿
 */

float G_offset = 0;

float Graverity_Offet(float angle)
{
    /*  重力补偿      */
    float rad = angle * M_PI / 180.0f;
    G_offset = -cos(rad) * 4255;
//    G_offset = -cos(rad) * Debug_Param().vel_kp;
    return G_offset;
}

float portSetPitch()
{
    static float portion = TIMpiece;
    static float tar_pos = 0;
    static float _tar_pos = 0;
    static int last_state = 0;
    static int direction = 1;
    switch (Car.CarType)
    {

        case cCar::ONMI:
            switch (Car.CtrlMode)
            {
                case cCar::eRC: //pitch遥控

//                    pitch_R.MotorCtrl.c_PID.setParam(45, 0.01, 0, 16384,
//                                                     40,0, 40, 300);
                    pitch_R.MotorCtrl.c_PID.setParam(270, 1, 0, 16384,
                                                     10,0, 0, 300);

                    _tar_pos += (float) RC_GetDatas().rc.ch[1] * portion;
                    //全步目标值限位
                    if (_tar_pos >= PITCH_MAX) _tar_pos = PITCH_MAX;
                    if (_tar_pos <= PITCH_MIN) _tar_pos = PITCH_MIN;
                    tar_pos = -_tar_pos;

                    break;
                case cCar::eKey: //pitch键盘
                    switch (Car.ShootMode)
                    {
                        case cCar::eNormal:

//                            pitch_R.MotorCtrl.c_SMC.SetParam(1.18,30,89,87,200,1.1,0.5,16350,TFSMC,0);///编码器参数
//                            pitch_R.MotorCtrl.c_SMC.SetParam(1.2,130,25,10,1.1,0.5,16350,EISMC,0);///编码器参数
//                            pitch_R.MotorCtrl.c_SMC.SetParam(1.5,250,20,1.1,0.5,16350,EXPONENT,0);///编码器参数
//                            pitch_R.MotorCtrl.c_SMC.SetParam(Debug_Param().vel_kp, Debug_Param().vel_ki,Debug_Param().vel_kd,Debug_Param().vel_rampTargetValue,Debug_Param().vel_maxIntegral,0.5, 16350, EISMC,0.0f);
//                            pitch_R.MotorCtrl.c_SMC.SetParam(1.18,48,14,14,1.1,0.5,16350,EISMC,0);///imu参数
//                            pitch_R.MotorCtrl.c_PID.setParam(45, 0.01, 0, 16384,
//                                                             40,0, 40, 300);
                            pitch_R.MotorCtrl.c_PID.setParam(270, 1, 0, 16384,
                                                             10,0, 0, 300);
                            _tar_pos -= (float) RC_GetDatas().mouse.y * mouse_sense;
                            if (_tar_pos >= PITCH_MAX) _tar_pos = PITCH_MAX;
//                            if (_tar_pos <= PITCH_MIN) _tar_pos = PITCH_MIN;
                            if (_tar_pos <= -8) _tar_pos = -8;

//                            tar_pos = -pitch_R.RCcrtl_filter.ckalman.Calc(_tar_pos);
                            tar_pos = -_tar_pos;
                            //全步目标值限位
                            break;
                        case cCar::eAutoaim:

//                            pitch_R.MotorCtrl.c_SMC.SetParam(1.5,250,20,1.1,0.5,16350,EXPONENT,0);///编码器参数
//                            pitch_R.MotorCtrl.c_SMC.SetParam(1.2,130,25,10,1.1,0.5,16350,EISMC,0);///编码器参数
//                            pitch_R.MotorCtrl.c_SMC.SetParam(1.2,360,4,14,1.1,0.5,16350,EISMC,0);///编码器参数


//                                pitch_R.MotorCtrl.c_SMC.SetParam(1.18,48,14,14,1.1,0.5,16350,EISMC,0);///imu参数
//                                if ( _tar_pos < 13 && _tar_pos > -1)
//                                    pitch_R.MotorCtrl.c_PID.setParam(45, 0.01, 0, 16384,
//                                                                     5,0.07, 40, 300);
//                                else if (_tar_pos >= 13)
//                                    pitch_R.MotorCtrl.c_PID.setParam(45, 0.01, 0, 16384,
//                                                                     12,0.07, 40, 300);
//                                else if (_tar_pos < -1 )
//                                    pitch_R.MotorCtrl.c_PID.setParam(45, 0.01, 0, 16384,
//                                                                     8,0.01, 40, 300);
                                pitch_R.MotorCtrl.c_PID.setParam(250, 5, 0, 16384,
                                                                 3,0, 2, 16384);


                            if (pitch_R.autoaimflag)
                            {
                                if (fabs(vision_pkt.offset_pitch) >= 0.3f)
                                {
//                                    _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Pitch) - vision_pkt.offset_pitch;
//                                    _tar_pos = (pitch_L.getEcd().total_angle - Car.getPCarPitchCentre() - vision_pkt.offset_pitch);///自瞄，待测
                                    _tar_pos = -vision_pkt.offset_pitch;
//                                    _tar_pos = (pitch_L.getEcd().total_angle - Car.getPCarPitchCentre()) - vision_pkt.offset_pitch;

                                }
                                pitch_R.autoaimflag = false;
                            }
                            if (_tar_pos >= PITCH_MAX) _tar_pos = PITCH_MAX;
                            if (_tar_pos <= PITCH_MIN) _tar_pos = PITCH_MIN;//                            if (_tar_pos <= -8) _tar_pos = -8;

//                    tar_pos = -pitch_R.autoAim_filtver.cLowPass.filter(_tar_pos);
                            tar_pos = -_tar_pos;

                            break;
                        case cCar::ePatrol:
                            pitch_R.MotorCtrl.c_PID.setParam(270, 1, 0, 16384,
                                                             10,0, 0, 300);
                            _tar_pos += direction * 0.02f;
                            if (_tar_pos >= PITCH_MAX-2) direction = -1;
                            if (_tar_pos <= PITCH_MIN+2) direction = 1;
                            tar_pos = -_tar_pos;
                            break;
                    }

                    break;
                case cCar::eRC_Autoaim: //pitch自瞄

//                    pitch_R.MotorCtrl.c_SMC.SetParam(1.5,250,20,1.1,0.5,16350,EXPONENT,0);///编码器参数
//                    pitch_R.MotorCtrl.c_SMC.SetParam(0.44,6,20,38,0.5,16350,EXPONENT,0);///编码器参数
//
//                    pitch_R.MotorCtrl.c_SMC.SetParam(Debug_Param().vel_kp, Debug_Param().vel_ki,Debug_Param().vel_kd,Debug_Param().vel_rampTargetValue,Debug_Param().vel_maxIntegral,0.5, 16350, EISMC,0.0f);
//
//                    pitch_R.MotorCtrl.c_SMC.SetParam(Debug_Param().vel_kp, Debug_Param().vel_ki,Debug_Param().vel_kd,Debug_Param().vel_rampTargetValue,0.5, 16350, EXPONENT,0.0f);
//
//                    pitch_R.MotorCtrl.c_SMC.SetParam(1.2,360,4,14,1.1,0.5,28000,EISMC,0);///编码器参数
//                    pitch_R.MotorCtrl.c_SMC.SetParam(Debug_Param().vel_kp, Debug_Param().vel_ki,Debug_Param().vel_kd,Debug_Param().vel_rampTargetValue,Debug_Param().vel_maxIntegral,0.5, 16350, EISMC,0.0f);

//                        pitch_R.MotorCtrl.c_SMC.SetParam(1.18,48,14,14,1.1,0.5,16350,EISMC,0);///imu参数
//                        pitch_R.MotorCtrl.c_PID.setParam(45, 0.01, 0, 16384,
//                                                         40,0, 40, 300);
//                        pitch_R.MotorCtrl.c_PID.setParam(45, 1, 0, 16384,
//                                                         20,0, 0, 300);
                    pitch_R.MotorCtrl.c_PID.setParam(250, 5, 0, 16384,
                                                     3,0, 2, 16384);
//                        if (_tar_pos < 13)
//                            pitch_R.MotorCtrl.c_PID.setParam(45, 0.01, 0, 16384,
//                                                             5,0.07, 40, 300);
//                        else if (_tar_pos >= 13)
//                            pitch_R.MotorCtrl.c_PID.setParam(45, 0.01, 0, 16384,
//                                                             12,0.07, 40, 300);



                    if (pitch_R.autoaimflag)
                    {
                        if (fabs(vision_pkt.offset_pitch) >= 0.3f)
                        {
//                            _tar_pos = IMU.cAngle(cimu::Wit_imu, cimu::Pitch) - vision_pkt.offset_pitch;
//                            _tar_pos = (pitch_L.getEcd().total_angle - Car.getPCarPitchCentre() - vision_pkt.offset_pitch);///
                            _tar_pos = - vision_pkt.offset_pitch;
//                            _tar_pos -= vision_pkt.offset_pitch;
//                            _tar_pos =(pitch_R.getEcd().total_angle -36)- vision_pkt.offset_pitch;
//                            _tar_pos = (pitch_L.getEcd().total_angle - Car.getPCarPitchCentre()) - vision_pkt.offset_pitch;
//                            _tar_pos = (pitch_L.getEcd().total_angle - Car.getPCarPitchCentre()) - vision_pkt.offset_pitch + (pitch_R.getEcd().total_angle -36);

                        }
                        pitch_R.autoaimflag = false;
                    }
                    if (_tar_pos >= PITCH_MAX) _tar_pos = PITCH_MAX;
                    if (_tar_pos <= PITCH_MIN) _tar_pos = PITCH_MIN;
//                    tar_pos = -pitch_R.autoAim_filter.cLowPass.filter(_tar_pos);
                    tar_pos = -_tar_pos;
//                    tar_pos = -pitch_R.autoAim_filter.ckalman.Calc(_tar_pos);
                    break;
            }
            break;
    }


    //pitch轴数据发送
//    usart_printf("%.2f,%.2f\r\n",  -pitch_R.getEcd().total_angle + Car.getPCarPitchCentre(),IMU.cAngle(cimu::Wit_imu, cimu::Pitch));
//    usart_printf("%.2f,%.2f,%.2f,%.2f\r\n", yaw.MotorCtrl.c_PID.pidOut(), yaw.getEcd().torque_current, pitch.MotorCtrl.c_PID.pidOut(),pitch.getEcd().torque_current);

//    usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", -tar_pos, -pitch_L.getEcd().total_angle + Car.getPCarPitchCentre(),IMU.cAngle(cimu::Wit_imu, cimu::Pitch),vision_pkt.offset_pitch,pitch_R.MotorCtrl.c_SMC.Out());
//    usart_printf("%.2f,%.2f,%.2f,%.2f\r\n",-(pitch_L.getEcd().total_angle - Car.getPCarPitchCentre()),vision_pkt.offset_pitch,_tar_pos,tar_pos);
//    tar_pos = -Debug_Param().vel_kp;

    Graverity_Offet(pitch_L.getEcd().total_angle - Car.getPCarPitchCentre());
//    Graverity_Offet(tar_pos);

//    float x = (float)pitch_L.Filter.cLowPass.filter(IMU.cAngle(cimu::Wit_imu, cimu::Pitch));
//    usart_printf("%.2f,%.2f,%.2f,%.2f,%.2f\r\n", -tar_pos, IMU.cAngle(cimu::Wit_imu, cimu::Pitch), -pitch_R.getEcd().total_angle + Car.getPCarPitchCentre(), vision_pkt.offset_pitch,pitch_R.MotorCtrl.c_SMC.Out());
//    usart_printf("%.2f,%.2f,%.2f\r\n",pitch_L.getEcd().torque_current,pitch_R.getEcd().torque_current,pitch_R.MotorCtrl.c_PID.pidOut());
    return tar_pos;
}
