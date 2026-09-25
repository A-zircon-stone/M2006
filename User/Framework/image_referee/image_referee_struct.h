//
// Created by ZONE7 on 2025/4/4.
//

#ifndef CUSTOM_CONTROLLER_TRASFER_IMAGE_REFEREE_H
#define CUSTOM_CONTROLLER_TRASFER_IMAGE_REFEREE_H
#include "stm32f4xx.h"
#include "stdio.h"
#include "string.h"

#define PROTOCAL_FRAME_MAX_SIZE 128 // 整个交互数据的包总共长最大为 128 个字节
#define FrameHeaderSOF 0xA5
#define C5_Remotec_Offset 1024
#define C5HeaderSof 0xA9
#define C5_ImageSof 0x53
#define Image_Referee_RecDataBuffDepth 200 // 接收缓存区单个通道深度，
#define Image_Referee_TxDataBuffDepth 200	 // DMA发送缓存区
extern uint8_t Image_flag;
#pragma pack (1)
// uint8_t Send_Pack_buf[Image_Referee_TxDataBuffDepth+1];
typedef __packed struct
{
    uint8_t SOF;		 // 数据帧起始字节，固定值为0xA5;
    uint16_t DataLength; // 数据帧内Data长度;
    uint8_t Seq;		 // 包序号;
    uint8_t CRC8;		 // 帧头CRC8;
} frame_header_t;		 // 一个完整的通信帧里面的帧头详细定义

typedef enum
{
    Clik_Once = 0,
    Clik_Hold = 1,
} click_mode_e;
typedef enum
{
    SELF_CONTROL_INTERACTIVE_ID = 0x0302, // 自定义控制器交互数据
    KEY_AND_MOUSE_ID = 0x0304,			  // 键鼠信息
    Clear_Flag = 0x00,
} frame_cmd_id_e;
typedef __packed struct
{
    frame_header_t header;
    frame_cmd_id_e frame_cmd_id;
    uint8_t Data[PROTOCAL_FRAME_MAX_SIZE];
    uint16_t frame_tail;
} RefereeFullFrame_t;
// 一个完整的通信帧格式

typedef __packed struct
{
    uint8_t header;
    uint8_t sof;
    uint8_t Data[PROTOCAL_FRAME_MAX_SIZE];
    uint16_t frame_tail;
} C5_Imgae_Frame_t;

typedef __packed struct {
    uint8_t Last_State: 1;
    uint8_t Now_State: 1;
    uint8_t Is_Click_Once: 1;       //“：1”声明后该变量只有0和1
    uint8_t Is_Click_Hold: 1;
    click_mode_e click_mode;
} Key_and_Mouse_t_click;
typedef __packed struct
{
    int16_t mouse_x;
    int16_t mouse_y;
    int16_t mouse_z;
    int8_t left_button_down;
    int8_t middle_button_down;
    int8_t right_button_down;
    uint16_t keyboard_value;
    uint16_t reserved;
} key_and_mouse_t;//图传链路客户端下发信息

#define self_controler_data_max_length 30 // 发送的内容数据段最大为 30
typedef __packed struct
{
    uint8_t Data[self_controler_data_max_length];
}self_control_data_t;//自定义控制器数据包结构体

typedef struct {
    int64_t live_time;
} C5_Online_detect_t;   //检测遥控器是否断连

typedef __packed struct
{
    __packed struct {
        int16_t mouse_x;//鼠标x轴，左负右正
        int16_t mouse_y;//鼠标y轴，上负下正
        int16_t mouse_z;//鼠标滚轮，前正后负
        Key_and_Mouse_t_click left_button_down;//左键
        Key_and_Mouse_t_click middle_button_down;
        Key_and_Mouse_t_click right_button_down;//右键
    } mouse;
    __packed struct{
        int16_t ch[5];
        uint8_t s[5];
        char Middle_Key;
        Key_and_Mouse_t_click Pause_Key;
        Key_and_Mouse_t_click User_Key_L;
        Key_and_Mouse_t_click User_Key_R;
        int16_t Rammc_Key;
        Key_and_Mouse_t_click Trigger_Key;
    } rc;
    __packed struct {
        uint16_t keyboard_value;
        Key_and_Mouse_t_click W;
        Key_and_Mouse_t_click S;
        Key_and_Mouse_t_click A;
        Key_and_Mouse_t_click D;
        Key_and_Mouse_t_click SHIFT;

        Key_and_Mouse_t_click CONTRL;
        Key_and_Mouse_t_click Q;
        Key_and_Mouse_t_click E;
        Key_and_Mouse_t_click R;
        Key_and_Mouse_t_click F;
        Key_and_Mouse_t_click G;
        Key_and_Mouse_t_click Z;
        Key_and_Mouse_t_click X;
        Key_and_Mouse_t_click C;
        Key_and_Mouse_t_click V;
        Key_and_Mouse_t_click B;
    } key;
    key_and_mouse_t Key_Mouse_raw;
} Image_ctrl_t;

typedef struct
{
    self_control_data_t self_control_all_data; // 自定义控制器命令交互数据
    key_and_mouse_t key_and_mouse;										   // 鼠标和键盘数据
} Image_receive_referee_t;
// 解包后获得的数据存在此结构体内

typedef struct
{
    uint32_t self_control_all_data; // 自定义控制器命令交互数据
    uint32_t key_and_mouse;						// 鼠标和键盘数据
} Image_Referee_UndateFlag_t;

#endif //CUSTOM_CONTROLLER_TRASFER_IMAGE_REFEREE_H

