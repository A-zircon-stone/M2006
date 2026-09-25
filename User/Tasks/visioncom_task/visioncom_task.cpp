//
// Created by mjw on 2023/1/12.
//
#include "visioncom_task.h"
#include "framework_headfile.hpp"
#include "packet.hpp"
#include "crc.h"
#include "usbd_cdc_if.h"
//#include "iwdgC.h"
//#include "INS_task.h"
#include "IMUC.hpp"
#include "gimbalc.hpp"
float Shoot_SpeedNow = 0;
int8_t MyColor, Fan_Type;
int32_t id = 0;

void Vision_JudgeUpdate(float shoot_speed, int8_t color, int8_t type)
{
    Shoot_SpeedNow = shoot_speed;
    MyColor = color;
    Fan_Type = type;
}

/*
	* @name   VisionChattingLoop
	* @brief  视觉通信循环
	* @param  mode 自瞄模式
	* @retval None
*/
SendPacket send_packet;
uint8_t Buf[sizeof(SendPacket)];

void VisionChattingLoop(uint8_t mode)
{
//    usart_printf("111\r\n");
//	float w = IMU_Quaternion().w;
//	float x = IMU_Quaternion().x;
//	float y = IMU_Quaternion().y;
//	float z = IMU_Quaternion().z;
//    float roll = IMU_NaiveAngle().roll;
//    float pitch = IMU_NaiveAngle().pitch; //用原始角好还是连续化之后的好？
//    float yaw = IMU_NaiveAngle().yaw;
//	if (Color_now == 1)	send_packet.color =0;
//	if (Color_now == 0)	send_packet.color =1;
    send_packet.header = 0x5A;

    send_packet.shoot_spd =refereeMsg.shoot_spd;
//    send_packet.shoot_spd =28;
//    send_packet.pitch = -pitch_R.getEcd().total_angle + Car.getPCarPitchCentre();
    send_packet.pitch = pitch_R.getEcd().total_angle - 36;//经验值，使URDF的pitch角度维持在正常范围
//    send_packet.pitch = -(pitch_L.getEcd().total_angle - Car.getPCarPitchCentre());//经验值，使URDF的pitch角度维持在正常范围

    send_packet.yaw = IMU.cAngle(cimu::Wit_imu, cimu::Yaw);
//    send_packet.roll = IMU_Angle(2);
    send_packet.roll = 0;
//    if (refereeMsg.our_color == 0)send_packet.enemy_color = true;  // our_color == 0 -> 自己是红色
//    else send_packet.enemy_color = false;
    if (refereeMsg.our_color == 0)send_packet.enemy_color = 'B';  // our_color == 0 -> 自己是红色
    else send_packet.enemy_color = 'R';
    send_packet.state = cCar::eRobot;
    send_packet.checksum = 0;


//    if(Car.EnergyMode == cCar::eSmallFu) //大小符
//    {
//        send_packet.energy_mechanism = 0;
//    }
//    else
//    {
//        send_packet.energy_mechanism = 1;
//    }
//    send_packet.enemy_color = 'B';


//	send_packet.packat_id = id;
    std::copy(reinterpret_cast<const uint8_t *>(&send_packet),
              reinterpret_cast<const uint8_t *>(&send_packet) + sizeof(SendPacket), Buf);
    Append_CRC16_Check_Sum(Buf, sizeof(SendPacket));
    CDC_Transmit_FS(Buf, sizeof(SendPacket));
//	usart_printf("%d\r\n",sizeof(SendPacket));
//	usart_printf("%f\r\n",Shoot_SpeedNow);
    id++;
    //这里放发送的函数
}

/*
	* @name   VisionComTask
	* @brief  视觉通信任务,1ms1次
	* @param  None
	* @retval None
*/
void VisionTask(void const *argument)
{
    /* USER CODE BEGIN VisionComTask */
    portTickType CurrentTime;

    /* Infinite loop */
    for (;;)
    {
//        FeedDog();
        CurrentTime = xTaskGetTickCount();
//		int8_t Zimiao = portIsZimiao(); //暂时不改
        VisionChattingLoop(1);
        vTaskDelayUntil(&CurrentTime, 5 / portTICK_RATE_MS);
    }
    /* USER CODE END VisionComTask */
}