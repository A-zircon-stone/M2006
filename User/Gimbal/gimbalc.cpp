  //
// Created by Andrew on 2024/8/27.
//
#include "gimbalc.hpp"
#include "math.h"
#include"DebugC.h"
#include "RC_Key.h"

cMotor yaw, pitch_R, pitch_L, yaw_v2;   //R-0X206 R->无人机pitch,L-0X207
cMotor fricR, fricL;
cMotor rammc;
cMotor car_follow_up;
Shootc shoot(&fricR, &fricL, &rammc);


/** Car **/             //choose your car here
  cCar Car(cCar::ONMI);
/**
 * ONMI 全向
 * MECANUM 麦步
 * UAV 无人机
 */

void Filter_Init()
{
    yaw.autoAim_filter.cLowPass.Init(15, 0.002);
    pitch_R.autoAim_filter.cLowPass.Init(10, 0.002);
    yaw.RCcrtl_filter.ckalman.Init(40, 200);
    pitch_R.RCcrtl_filter.ckalman.Init(40, 200);
    pitch_L.RCcrtl_filter.ckalman.Init(40, 200);
}

void Algorithm_Init()
{
    yaw.MotorCtrl.c_PID.Init(); //pid参数初始化
    yaw.MotorCtrl.c_PID.setParam(0,0,0,16384,
                                 0,0,0,300);//在此处设置PID参数
    yaw.MotorCtrl.c_SMC.Init();

    yaw_v2.MotorCtrl.c_PID.Init();
    yaw_v2.MotorCtrl.c_PID.setParam(0,0,0,2000,
                                    0,0,0,300);

    car_follow_up.MotorCtrl.c_PID.Init();//跟随

    fricL.MotorCtrl.c_PID.Init();
    fricL.MotorCtrl.c_PID.setParam(20,0,0,16384,
                                   20,0,0,400);
    fricR.MotorCtrl.c_PID.Init();
    fricR.MotorCtrl.c_PID.setParam(20,0,0,16384,
                                   20,0,0,400);

    rammc.MotorCtrl.c_PID.Init();
    rammc.MotorCtrl.c_PID.setParam(0,0,0,10000,
                                   0,0,0,500);
    rammc.MotorCtrl.c_SMC.Init();
    rammc.MotorCtrl.c_SMC.SetParam(0.3, 2000,2,10,1.1, 0.5, 9600, EISMC,0.0f);


    pitch_R.MotorCtrl.c_PID.Init();
//    pitch_R.MotorCtrl.c_PID.setParam(20,0,0,16384,
//                                     20,0,0,300);
////    pitch_L.MotorCtrl.c_PID.Init();


}

/**
 * 电机初始化
 * canID can口 电机型号
 */
void MotorInit()
{
    yaw.canInit(0x205,&hcan2,cMotor::GM_6020_cur);
//    yaw_v2.canInit(0x141,&hcan1,cMotor::MF_9025);
    fricL.canInit(0x201, &hcan1, cMotor::M_3508); //请自行查询电机反馈报文ID，信息来源https://www.robomaster.com/zh-CN/products/components/general?djifrom=nav
    fricR.canInit(0x202,&hcan1,cMotor::M_3508);
    rammc.canInit(0x203,&hcan1,cMotor::M_2006_p36);
    pitch_R.canInit(0x206, &hcan1, cMotor::GM_6020_cur);
    pitch_L.canInit(0x207, &hcan1, cMotor::GM_6020_cur);
}

/**
 * 底盘初始化
 *
 */
  void ChassisInit()
  {
      CHASSIS_YAW.send_flag = 1;   //1 进行底盘控制 0 不进行
      CHASSIS_VEL.send_flag = 1;
  }

  bool CarInit()//
  {
      Car.CtrlMode = cCar::eRC;   //初始化车的遥控方式
      Car.is_protect = true;
      return false;
  }

/**
 * Gimbal所有功能初始化
 */
void Gimbal_Init()
{
    MotorInit();
    Filter_Init();
    Algorithm_Init();
    ChassisInit();
    CarInit();
}

/**********************************RC_Ctrl**********************************************/

/**
 * 保护检测 如果遥控器指定保护或遥控器断连
 * @return Car.is_protect true
 */
bool portSetProtect()
{
    RC_GetNewData.live_time--;
    if (RC_GetNewData.live_time <= 0)
        Car.is_protect = true;
    else Car.is_protect = false;
    return Car.is_protect;
}

/**
* 发送云台电机绝对编码器相对偏移 实现随动
* @param yaw_offset
*/
void sendChassisYaw(float yaw_offset)
{
    CHASSIS_YAW.CANx_send_data[0] = (int16_t) (yaw_offset) >> 8;
    CHASSIS_YAW.CANx_send_data[1] = (int16_t) (yaw_offset) & 0xff;
}

/**
 * 控制小陀螺
 * @param spinVel 小陀螺速度
 */
  void setSpin(float spinVel)
  {
      CHASSIS_VEL.CANx_send_data[4] = (int16_t) (spinVel) >> 8;
      CHASSIS_VEL.CANx_send_data[5] = (int16_t) (spinVel) & 0xff;
      CHASSIS_VEL.CANx_send_data[6] = 2;
  }

/**
 * 底盘保护模式 让车停下
 */
void portSetChassicStop()
{
    int16_t vx, vy, spinVel;
    vx = 0;
    vy = 0;
    spinVel = 0;
    CHASSIS_VEL.CANx_send_data[0] = vx >> 8;
    CHASSIS_VEL.CANx_send_data[1] = vx & 0xff;
    CHASSIS_VEL.CANx_send_data[2] = vy >> 8;
    CHASSIS_VEL.CANx_send_data[3] = vy & 0xff;
    CHASSIS_VEL.CANx_send_data[4] = spinVel >> 8;
    CHASSIS_VEL.CANx_send_data[5] = spinVel & 0xff;
    CHASSIS_VEL.CANx_send_data[6] = 5;      //PROTECT mode
    CHASSIS_VEL.sendBuff();
}


  void getCtrlData()
{
    RC_Levers();
    portSetMove();
    pitch_R.MotorCtrl.c_PID.setPosTar(portSetPitch());
//    yaw.MotorCtrl.c_PID.setPosTar(portSetYaw());
    yaw.MotorCtrl.c_SMC.ErrorUpdate(portSetYaw(), IMU.cAngle(cimu::Wit_imu, cimu::Yaw),
                                    IMU.cSpeed(cimu::Wit_imu, cimu::Yaw));
//    yaw_v2.MotorCtrl.c_PID.setPosTar(portSetYaw());
    shoot.SetShootLoop(Car);

}

//静态pitch编码器角度跳变
//  float Pitch_ecd_static_offset()
//  {
//      static bool flag = true;
//      static uint16_t offset = 0;
//      if (flag)
//      {
//          flag = false;
//          if (fricL.getEcd().angle > 100)
//              offset = 360;
//          else offset = 0;
//      }
//      return offset;
//  }

/**********************************Motor spdLoop and send current**********************************************/
/**
 * 电机所有算法计算
 */
void Can_Calc()
{

    fricL.MotorCtrl.c_PID.spdLoop(fricL.getEcd().speed); //速度环
    fricR.MotorCtrl.c_PID.spdLoop(fricR.getEcd().speed); //速度环
//    rammc.MotorCtrl.c_PID.spdLoop(rammc.getEcd().speed);
//    rammc.MotorCtrl.c_PID.posLoop(rammc.getEcd().total_angle,rammc.getEcd().speed);
      rammc.MotorCtrl.c_SMC.SmcCalculate();
//    yaw.MotorCtrl.c_PID.spdLoop(IMU.cSpeed(cimu::Wit_imu,cimu::Yaw));
//    yaw.MotorCtrl.c_PID.posLoop(yaw.getEcd().total_angle,yaw.getEcd().speed);
//    yaw.MotorCtrl.c_PID.posLoop(IMU.cAngle(cimu::Wit_imu,cimu::Yaw),
//                                IMU.cSpeed(cimu::Wit_imu,cimu::Yaw));
    yaw.MotorCtrl.c_SMC.SmcCalculate();
//    yaw_v2.MotorCtrl.c_PID.spdLoop(yaw_v2.getEcd().speed);
//    yaw_v2.MotorCtrl.c_PID.posLoop(yaw_v2.getEcd().total_angle,yaw_v2.getEcd().speed);
    pitch_R.MotorCtrl.c_PID.posLoop(-(pitch_L.getEcd().total_angle - Car.getPCarPitchCentre()),
                                    -(pitch_L.getEcd().speed));
//    pitch_R.MotorCtrl.c_PID.posLoop(IMU.cAngle(cimu::Wit_imu,cimu::Pitch),IMU.cSpeed(cimu::Wit_imu,cimu::Pitch));

    /**
     * 随动模式 计算
     */
    if (portSetFollowUp())
    {
        if (Car.getPCarInEcd() + Car.getPCarEcdOffset() - Car.getPCarAngleOffset() -Car.getPCarAngleOffsetturn()- yaw.getEcd().total_angle > 180)Car.setPCarEcdOffset(-360); //加减2π
        if (yaw.getEcd().total_angle - Car.getPCarInEcd() - Car.getPCarEcdOffset() + Car.getPCarAngleOffset()+Car.getPCarAngleOffsetturn() > 180)Car.setPCarEcdOffset(360);//优弧劣弧处理
        car_follow_up.MotorCtrl.c_PID.setPosTar(Car.getPCarInEcd() + Car.getPCarEcdOffset() - Car.getPCarAngleOffset()-Car.getPCarAngleOffsetturn());
        car_follow_up.MotorCtrl.c_PID.setParam(1.8, 0, 22, 1000);
        float chassisyaw = -car_follow_up.MotorCtrl.c_PID.posLoop(yaw.getEcd().total_angle);
        setSpin(chassisyaw);
        CHASSIS_VEL.CANx_send_data[6] = 3;
    }
}

void Can_Send()
{
//    yaw.canSend(cMotor::ePid);
    yaw.canSend(cMotor::eExternal, (int16_t)yaw.MotorCtrl.c_SMC.Out());

//    yaw_v2.canSend(cMotor::eExternal,500);
//    yaw_v2.canSend(cMotor::ePid);
    fricL.canSend(cMotor::ePid);
    fricR.canSend(cMotor::ePid);
    if (Car.FricCtrlMode == cCar::eFric_SHUT)  // 摩擦轮关闭，设置摩擦轮速度为0
    {
        if (fabs(fricL.getEcd().speed) <= 500) fricL.protect();
        if (fabs(fricR.getEcd().speed) <= 500) fricR.protect();
    }
//    rammc.canSend(cMotor::ePid);
    rammc.canSend(cMotor::eExternal, rammc.MotorCtrl.c_SMC.Out());
//    pitch_R.canSend(cMotor::ePid);
//    pitch_L.canSend(cMotor::ePid);
    pitch_R.canSend(cMotor::eExternal, pitch_R.MotorCtrl.c_PID.pidOut() + G_offset);           //DONE:重力补偿
    pitch_L.canSend(cMotor::eExternal, -pitch_R.MotorCtrl.c_PID.pidOut() - G_offset);
    sendChassisYaw(-(yaw.getEcd().total_angle - (Car.getPCarInEcd() + Car.getPCarEcdOffset())));   //发送陀螺或者随动模式的数据

}

void CanMxg()
{
    uint8_t i;
    for (i = 0; i <= 13; i++)
    {
        CanReg[i].sendBuff();
    }
}

void GimbalLoop()
{
    if (portSetProtect())
    {
        Car.CtrlMode = cCar::eProtect;
        portSetYaw();
        yaw.protect();
        pitch_R.protect();
        pitch_L.protect();
        fricL.MotorCtrl.c_PID.spdLoop(fricL.getEcd().speed);
        fricR.MotorCtrl.c_PID.spdLoop(fricR.getEcd().speed);
        fricL.MotorCtrl.c_PID.setSpdTar(0);
        fricR.MotorCtrl.c_PID.setSpdTar(0);
        if (fabs(fricR.getEcd().speed) <= 500) fricR.protect();
        else fricR.canSend(cMotor::ePid);
        if (fabs(fricL.getEcd().speed) <= 500) fricL.protect();
        else fricL.canSend(cMotor::ePid);
        shoot.SetRammcShoot(Car);
        rammc.protect();
        IMU_Clean_Rotate();
        yaw.clearEcdRcnt();
        portSetChassicStop();
        UI_Protect(LED_ON);
    }
    else
    {
        getCtrlData();
        Can_Calc();
        Can_Send();
//        CHASSIS_YAW.CANx_send_data[5] = 1;
//        CHASSIS_YAW.CANx_send_data[2] = 1;
        UI_Protect(LED_OFF);
    }

    CanMxg();
}

void PrintTest()
{
//    usart_printf("111\r\n");
//        usart_printf("%d,%d,%d,%d,%d\r\n",RC_GetDatas().rc.ch[0],RC_GetDatas().rc.ch[1],RC_GetDatas().rc.ch[2],RC_GetDatas().rc.ch[3],RC_GetDatas().rc.ch[4]);
//    usart_printf("%f,%f\r\n",IMU.cAngle(cimu::Wit_imu,cimu::Yaw),portSetYaw());
//    usart_printf("%f\r\n",portSetPitch());
//    usart_printf("%f\r\n",pitch_R.MotorCtrl.c_PID.getPosParam().PID_Target);
//    usart_printf("%f,%f,%d\r\n",yaw.MotorCtrl.c_PID.getPosParam().PID_Target,IMU.cAngle(cimu::Wit_imu,cimu::Yaw),RC_GetDatas().rc.ch[0]);
//    usart_printf("%f,%f,%f\r\n",yaw.getEcd().angle,yaw.getEcd().total_angle,IMU.cAngle(cimu::Wit_imu,cimu::Yaw));
//    usart_printf("%f\r\n",yaw.getEcd().torque_current);
//    usart_printf("%f,%f\r\n",IMU.cAngle(cimu::Wit_imu,cimu::Yaw),yaw.MotorCtrl.c_SMC.getSmc().error.tar_now);
//    usart_printf("%f,%f\r\n",portSetPitch(),-(pitch_L.getEcd().total_angle - Car.getPCarPitchCentre()));
//    usart_printf("%f,%f,%d,%f\r\n",rammc.getEcd().total_angle,rammc.getEcd().speed,RC_GetDatas().rc.ch[4],rammc.MotorCtrl.c_PID.getSpdTar());
//    usart_printf("%f,%f,%f,%f\r\n",rammc.getEcd().speed,rammc.MotorCtrl.c_PID.getSpdTar(),rammc.getEcd().total_angle,rammc.MotorCtrl.c_PID.getPosParam().PID_Target);
//    usart_printf("%f,%f\r\n",rammc.getEcd().total_angle,rammc.MotorCtrl.c_PID.getPosParam().PID_Target);
//    usart_printf("%f\r\n",rammc.getEcd().torque_current);
//    usart_printf("%f,%f\r\n",rammc.getEcd().total_angle,rammc.MotorCtrl.c_SMC.getSmc().error.tar_now);
//    usart_printf("%.2f,%.2f,1,%.2f\r\n",rammc.getEcd().total_angle,rammc.MotorCtrl.c_SMC.getSmc().error.tar_now,rammc.getEcd().torque_current);
//    usart_printf("%f,%f\r\n",fricL.getEcd().speed,fricR.getEcd().speed);
//    usart_printf("%f,%f\r\n",fricL.MotorCtrl.c_PID.getSpdTar(),fricR.MotorCtrl.c_PID.getSpdTar());
//    usart_printf("%f,%f,%f\r\n",pitch_R.MotorCtrl.c_PID.getPosParam().PID_Target,pitch_L.MotorCtrl.c_PID.getSpdParam().PID_Target,-(pitch_L.getEcd().total_angle - Car.getPCarPitchCentre()));
//    usart_printf("%f,%f,%f,%f\r\n",IMU.cAngle(cimu::Wit_imu,cimu::Yaw),yaw.MotorCtrl.c_SMC.getSmc().error.tar_now,pitch_R.getEcd().total_angle,pitch_R.MotorCtrl.c_PID.getPosParam().PID_Target);
//    usart_printf("Last_State:%d,Now_State:%d,Is_Click_Once:%d\r\n",RC_GetDatas().key.CONTRL.Last_State,RC_GetDatas().key.CONTRL.Now_State,RC_GetDatas().key.CONTRL.Is_Click_Once);
//    usart_printf("%d,%d,%d\r\n",image_ctrl.rc.Trigger_Key.Now_State,image_ctrl.rc.Trigger_Key.Last_State,image_ctrl.rc.Trigger_Key.Is_Click_Once);
//    usart_printf("%d,%d,%d\r\n",image_ctrl.rc.User_Key_L.Now_State,image_ctrl.rc.User_Key_L.Last_State,image_ctrl.rc.User_Key_L.Is_Click_Once);
//    usart_printf("%d,%d,%d\r\n",image_ctrl.rc.Pause_Key.Now_State,image_ctrl.rc.Pause_Key.Last_State,image_ctrl.rc.Pause_Key.Is_Click_Once);
//    usart_printf("%d,%d,%d,%d,%d\r\n",image_ctrl.rc.ch[0],image_ctrl.rc.ch[1],image_ctrl.rc.ch[2],image_ctrl.rc.ch[3],image_ctrl.rc.ch[4]);
//    usart_printf("%d,%d,%d,%d\r\n",image_ctrl.mouse.middle_button_down.Now_State,image_ctrl.mouse.middle_button_down.Last_State,image_ctrl.mouse.middle_button_down.Is_Click_Once,image_ctrl.mouse.middle_button_down.Is_Click_Hold);
//    usart_printf("%f,%f\r\n",yaw.MotorCtrl.c_SMC.getSmc().error.tar_now,IMU.cAngle(cimu::Wit_imu,cimu::Yaw));
//    usart_printf("%d,%d,%d\r\n",RC_GetDatas().key.B.Now_State,RC_GetDatas().key.B.Last_State,RC_GetDatas().key.B.Is_Click_Once);
//    usart_printf("%d,%f,%d,%f\r\n",image_ctrl.rc.ch[0],yaw.MotorCtrl.c_SMC.getSmc().error.tar_now,image_ctrl.rc.ch[1],pitch_R.MotorCtrl.c_PID.getPosParam().PID_Target);
//    usart_printf("%d\r\n",RC_GetDatas().mouse.press_r.Now_State);
//    usart_printf("%d\r\n",Car.ShootMode);
}

/**
 * CarCanMxg() 发送底盘数据
 */
  void CarCanMxg()
  {
      if (portSetProtect())
      {

      }
      else
      {
          static uint8_t send_flag = 0;
          if (send_flag)
          {
              CHASSIS_VEL.sendBuff();
              send_flag = 0;
          }
          else
          {
              CHASSIS_YAW.sendBuff();
              send_flag = 1;
          }
      }
  }

