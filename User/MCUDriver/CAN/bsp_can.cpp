//
// Created by Andrew on 2024/8/27.
//

#include "bsp_can.hpp"
#include "main.h"
#include "gimbalc.hpp"
#include "MotorC.hpp"
#include "ShooterMotors.hpp"


extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;
//DJI 底盘通信
CanType  CanReg[] = {CanType(&hcan1, 0x1FF), //motor //0
                    CanType(&hcan1, 0x200), //motor
                    CanType(&hcan1, 0x2FF), //motor
                    CanType(&hcan2, 0x1FF), //motor
                    CanType(&hcan2, 0x200), //motor
                    CanType(&hcan2, 0x2FF), //can包
                    CanType(&hcan2, 0x401), //chassis vel
                    CanType(&hcan2, 0x402), //chassis yaw
                    CanType(&hcan1, 0x601),  //chassis yaw
                    CanType(&hcan1, 0x1FE), //motor
                    CanType(&hcan1, 0x2FE), //motor //10
                    CanType(&hcan2, 0x1FE), //can包
                    CanType(&hcan2, 0x2FE), //can包
                    CanType(&hcan1,0x141),//MF9025
};


void can_filter_init(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef can_filter_st;                                                            //定义过滤器结构体
    can_filter_st.FilterActivation = ENABLE;                                                    //ENABLE使能过滤器
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;                                           //设置过滤器模式--标识符屏蔽位模式
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;                                          //过滤器的位宽 32 位
    can_filter_st.FilterIdHigh = 0x0000;                                                        //ID高位
    can_filter_st.FilterIdLow = 0x0000;                                                         //ID低位
    can_filter_st.FilterMaskIdHigh = 0x0000;                                                    //过滤器掩码高位
    can_filter_st.FilterMaskIdLow = 0x0000;                                                     //过滤器掩码低位
    can_filter_st.SlaveStartFilterBank = 14;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;                                          //与过滤器组管理的 FIFO
    if (hcan->Instance == CAN1)
    {
        can_filter_st.FilterBank = 0;
    }
    else if (hcan->Instance == CAN2)
    {
        can_filter_st.FilterBank = 14;
    }
    HAL_CAN_ConfigFilter(hcan, &can_filter_st);                                     //HAL库配置过滤器函数
    HAL_CAN_Start(hcan);                                                                            //使能CAN控制器
    __HAL_CAN_ENABLE_IT(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);                    //使能CAN的各种中断
}

void MotorMeasureFun(Motor_measure_t *ptr, uint8_t *RX_buffer)
{
    ptr->last_angle = ptr->angle;                                                       //记录上一次转子机械角度
    ptr->angle = (int16_t) ((RX_buffer)[0] << 8 | (RX_buffer)[1]);                     //解析转子机械角度
    ptr->speed = (int16_t) ((RX_buffer)[2] << 8 | (RX_buffer)[3]);                     //解析转子转速(rpm)
    ptr->torque_current = (int16_t) ((RX_buffer)[4] << 8 | (RX_buffer)[5]);            //解析转矩电流
    ptr->temp = (RX_buffer)[6];

    if (ptr->angle - ptr->last_angle > 4096)                                        //利用转子本次机械角度与上次机械角度的差值
        ptr->round_cnt--;                                                           //判断电机转子正向或反向过零
    else if (ptr->angle - ptr->last_angle < -4096)                                  //从而计算得到转子从上电开始转动的总圈数
        ptr->round_cnt++;

    ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle;                          //利用总圈数计算转子总角度
    //(出轴总角度要根据减速比再换算

}

//瓴控电机
void MotorMeasureFun_lk(Motor_measure_t *ptr, uint8_t *RX_buffer)
{
    ptr->last_angle = ptr->angle;                                                       //记录上一次转子机械角度
    ptr->angle = (int16_t) ((RX_buffer)[7] << 8 | (RX_buffer)[6]);                     //解析转子机械角度
    ptr->speed = (int16_t) ((RX_buffer)[5] << 8 | (RX_buffer)[4]);                     //解析转子转速(rpm)
    ptr->torque_current = (int16_t) ((RX_buffer)[3] << 8 | (RX_buffer)[2]);            //解析转矩电流
    ptr->temp = (RX_buffer)[1];

    if (ptr->angle - ptr->last_angle > 32768)                                        //利用转子本次机械角度与上次机械角度的差值
        ptr->round_cnt--;                                                           //判断电机转子正向或反向过零
    else if (ptr->angle - ptr->last_angle < -32768)                                  //从而计算得到转子从上电开始转动的总圈数
        ptr->round_cnt++;

    ptr->total_angle = ptr->round_cnt * 65535 + ptr->angle;                          //利用总圈数计算转子总角度
    //(出轴总角度要根据减速比再换算
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;                                                                //定义数据帧的帧头
    uint8_t rx_buffer[8];                                                                         //接收存放数据帧数据的数组

    HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_buffer);         //把can接收到的数据帧传入局部变量
    uint8_t rx_data[8];
    static int16_t vel_id[4] = {};
    static uint8_t i = 0;
    if (hcan == &hcan1)
    {
        pitch_R.canRead(rx_header.StdId, rx_buffer);
        pitch_L.canRead(rx_header.StdId, rx_buffer);
        fricL.canRead(rx_header.StdId, rx_buffer);
        fricR.canRead(rx_header.StdId, rx_buffer);
        rammc.canRead(rx_header.StdId, rx_buffer);
        yaw_v2.canRead(rx_header.StdId,rx_buffer);

        // 最小发射单元的三台电机也使用同一条 CAN1 反馈总线。
        shooter_fric_left.canRead(rx_header.StdId, rx_buffer);
        shooter_fric_right.canRead(rx_header.StdId, rx_buffer);
        shooter_feeder.canRead(rx_header.StdId, rx_buffer);
    }
    else if (hcan == &hcan2)
    {                //can2
        yaw.canRead(rx_header.StdId, rx_buffer);
//        yaw_v2.canRead(rx_header.StdId,rx_buffer);
        RefereeMsg(rx_header.StdId ,rx_buffer);

    }
}

void CanType::sendBuff()
{
    if (send_flag)
    {

        CANx_tx_message.StdId = _std_id;
        CANx_tx_message.IDE = CAN_ID_STD;                                                             //标识符选择位，STD-标准帧
        CANx_tx_message.RTR = CAN_RTR_DATA;                                                           //定义帧类型
        CANx_tx_message.DLC = 0x08;
        HAL_CAN_AddTxMessage(_hcan,
                             &CANx_tx_message,                           //hal库can发送函数：该函数用于向发送邮箱
                             CANx_send_data, &send_mail_box);            //添加发送报文，并激活发送请求
    }
    else;
}

CanType::CanType(CAN_HandleTypeDef *hcan, uint32_t std_id) : _hcan(hcan), _std_id(std_id)
{
    send_flag = 0;
}
