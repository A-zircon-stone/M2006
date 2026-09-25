//
// Created by ZONE7 on 2025/4/4.
//

#include "image_referee.h"
#include "image_referee_struct.h"
#include "usart.h"
#include "RemoteC.h"
#include "LEDC.h"
//#include "chassisc.hpp"




// cImage_Referee Image_Referee(&huart6, Image_Referee_RecDataBuffDepth, DMA_IDLE_IT);
uint8_t Image_Referee_RxBuf[Image_Referee_RecDataBuffDepth-1] = {0};
uint8_t Image_flag;

void Image_Referee_UartInit(void)
{
    __HAL_UART_ENABLE_IT(&huart6, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart6, Image_Referee_RxBuf,Image_Referee_RecDataBuffDepth );
}


Image_receive_referee_t Image_referee_recv_mesg;//解包获得的最终数据存在这儿
Image_Referee_UndateFlag_t	Image_Referee_UpdateFlag = {0,0};

static uint32_t DataRecLength = 0;//因为这个变量每次进中断都要用，所以声明为static，不需要每次进来都创建变量，节省一个机器周期


void Image_Referee_Receive_Data(void)
{

     if (__HAL_UART_GET_FLAG(&huart6, UART_FLAG_IDLE) != RESET)
     {
//         Image_flag = 1;
        __HAL_UART_CLEAR_IDLEFLAG(&huart6);
        HAL_UART_DMAStop(&huart6);
         HAL_GPIO_WritePin(GPIOH, GPIO_PIN_10, GPIO_PIN_SET);

//         static uint8_t *Referee_Pack = NULL;
//        Referee_Pack = Referee_Check_Frame(Image_Referee_RxBuf);
        uint8_t rx_len = Image_Referee_RecDataBuffDepth - __HAL_DMA_GET_COUNTER(&hdma_usart6_rx);
        __HAL_DMA_SET_COUNTER(&hdma_usart6_rx, Image_Referee_RecDataBuffDepth);
        if(Image_Referee_RxBuf[0]== C5HeaderSof)
        {
            C5_Image_Referee_UnpackRecData(Image_Referee_RecDataBuffDepth,Image_Referee_RxBuf);
        }
//         Image_Referee_UnpackRecData(Image_Referee_RecDataBuffDepth,Image_Referee_RxBuf);

        memset(Image_Referee_RxBuf, 0, rx_len);
        rx_len = 0 ;
        HAL_UART_Receive_DMA(&huart6, Image_Referee_RxBuf, Image_Referee_RecDataBuffDepth);
     }
}

/**
 * @brief 裁判系统协议校验
 * @param StartByte
 * @return 校验完成数据包帧头
 */
uint8_t* Referee_Check_Frame(uint8_t* StartByte)
{
    static uint8_t *Start_Flag_Byte = NULL;//储存帧首地址位置
    static uint8_t CRC8_Result = 0;
    static uint16_t CRC16_Result = 0;
    Start_Flag_Byte = (uint8_t*)strchr((const char*)StartByte,0xA5);//字符串内寻找第0xA5，返回其地址
    if(Start_Flag_Byte == NULL)//未接收到开始帧返回NULL
    {
        return NULL;
    }
    else
    {
        CRC8_Result = GetCRC8CheckSum(Start_Flag_Byte,4,CRC8_INIT_NUM);//CRC包头校验
        if(CRC8_Result == Start_Flag_Byte[4])
        {
            CRC16_Result = GetCRC16CheckSum(Start_Flag_Byte,7 + (uint16_t)*(uint16_t*)&Start_Flag_Byte[1],CRC16_INIT_NUM);
            if(CRC16_Result == (uint16_t)*(uint16_t*)&Start_Flag_Byte[ 7 + (uint16_t)*(uint16_t*)&Start_Flag_Byte[1] ])//CRC包尾校验
            {
                return Start_Flag_Byte;
            }
        }
    }
}

uint8_t* Referee_Check_Frame_C5(uint8_t* StartByte)
{
    static uint8_t *Start_Flag_Byte = NULL;//储存帧首地址位置
    static uint16_t CRC16_Result = 0;
    Start_Flag_Byte = (uint8_t*)strchr((const char*)StartByte,0xA9);//字符串内寻找第0xA5，返回其地址
    if(Start_Flag_Byte == NULL)//未接收到开始帧返回NULL
    {
        return NULL;
    }
    else
    {
        if(Start_Flag_Byte[1] == C5_ImageSof)
        {
            CRC16_Result= GetCRC16CheckSum(Start_Flag_Byte,19,CRC16_INIT_NUM);
            if(CRC16_Result == (uint16_t)*(uint16_t*)&Start_Flag_Byte[19])//CRC包尾校验
            {
                return Start_Flag_Byte;
            }
            return NULL;
        }
        return NULL;
    }
}

void C5_Image_Referee_UnpackRecData(uint16_t DataLength,uint8_t* buf_Data)
{
    // usart_printf("DataLength:%d\r\n",DataLength);
    static uint8_t *Referee_Pack = NULL;
    Referee_Pack = Referee_Check_Frame_C5(buf_Data);
    while (Referee_Pack != NULL)
    {
            switch ( ((C5_Imgae_Frame_t*)Referee_Pack)->sof )
            {

            case C5_ImageSof:
                {
                    if(Referee_Pack==NULL || buf_Data == NULL)
                    {
                        return;
                    }
                    C5_GetNewData.live_time = 100;
                    image_ctrl.rc.ch[0] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[0] | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[1] << 8)& 0x07ff;
                    image_ctrl.rc.ch[1] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[1] >> 3 | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[2] << 5)& 0x07ff;
                    image_ctrl.rc.ch[2] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[2] >> 6 | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[3] << 2 |
                                                        (( C5_Imgae_Frame_t* )Referee_Pack)->Data[4] << 10 )& 0x07ff;
                    image_ctrl.rc.ch[3] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[4] >> 1 | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[5] << 7)& 0x07ff;
                    image_ctrl.rc.s[0] =  ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[5] >> 4) & 0x0003;
                    image_ctrl.rc.s[1] =  ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[5] >> 6) & 0x0001;
                    image_ctrl.rc.s[2] =  ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[5] >> 7) & 0x0001;
                    image_ctrl.rc.s[3] =  (( C5_Imgae_Frame_t* )Referee_Pack)->Data[6] & 0x0001;
                    image_ctrl.rc.ch[4] = (int16_t)(( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[6] >> 1 | (( C5_Imgae_Frame_t* )Referee_Pack)->Data[7] << 7)& 0x07ff;
                    image_ctrl.rc.s[4] =  ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[7] >> 4) & 0x0001;
                    image_ctrl.Key_Mouse_raw.mouse_x = (int16_t)( ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[8]  | ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[9] << 8) & 0xffff;
                    image_ctrl.Key_Mouse_raw.mouse_y = (int16_t)( ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[10]  | ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[11] << 8) & 0xffff;
                    image_ctrl.Key_Mouse_raw.mouse_z = (int16_t)( ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[12]  | ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[13] << 8) & 0xffff;
                    image_ctrl.Key_Mouse_raw.left_button_down = ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[14] & 0x0001;
                    image_ctrl.Key_Mouse_raw.right_button_down = ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[14]>> 2) & 0x0001;
                    image_ctrl.Key_Mouse_raw.middle_button_down = ((( C5_Imgae_Frame_t* )Referee_Pack)->Data[14] >> 4) & 0x0001;
                    image_ctrl.Key_Mouse_raw.keyboard_value = (int16_t)( ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[15]  | ( ( C5_Imgae_Frame_t* )Referee_Pack)->Data[16] << 8) & 0xffff;

//                image_ctrl.rc.ch[0] = (int16_t)( Referee_Pack[2] | (Referee_Pack[3] << 8))& 0x07ff;
//                image_ctrl.rc.ch[1] = (int16_t)( Referee_Pack[3] >> 3 | (Referee_Pack[4] << 5))& 0x07ff;
//                image_ctrl.rc.ch[2] = (int16_t)(Referee_Pack[4] >> 6 | Referee_Pack[5] << 2 |
//                        Referee_Pack[6] << 10 )& 0x07ff;
//                image_ctrl.rc.ch[3] = (int16_t)(Referee_Pack[6] >> 1 | Referee_Pack[7] << 7)& 0x07ff;
//                image_ctrl.rc.s[0] =  (Referee_Pack[7] >> 4) & 0x0003;
//                image_ctrl.rc.s[1] =  (Referee_Pack[7] >> 6) & 0x0001;
//                image_ctrl.rc.s[2] =  (Referee_Pack[7] >> 7) & 0x0001;
//                image_ctrl.rc.s[3] =  (Referee_Pack[8]) & 0x0001;
//                image_ctrl.rc.ch[4] = (int16_t)(Referee_Pack[8] >> 1 | Referee_Pack[9] << 7)& 0x07ff;
//                image_ctrl.rc.s[4] =  (Referee_Pack[9] >> 4) & 0x0001;
//                image_ctrl.Key_Mouse_raw.mouse_x = (int16_t)( Referee_Pack[10]  | Referee_Pack[11]) & 0xffff;
//                image_ctrl.Key_Mouse_raw.mouse_y = (int16_t)( Referee_Pack[12]  | Referee_Pack[13] << 8) & 0xffff;
//                image_ctrl.Key_Mouse_raw.mouse_z = (int16_t)( Referee_Pack[14]  | Referee_Pack[15] << 8) & 0xffff;
//                image_ctrl.Key_Mouse_raw.left_button_down =  Referee_Pack[16] & 0x0001;
//                image_ctrl.Key_Mouse_raw.right_button_down = (Referee_Pack[16]>> 2) & 0x0001;
//                image_ctrl.Key_Mouse_raw.middle_button_down = (Referee_Pack[16] >> 4) & 0x0001;
//                image_ctrl.Key_Mouse_raw.keyboard_value = (int16_t)( Referee_Pack[17]  | Referee_Pack[18] << 8) & 0xffff;

                    image_ctrl.rc.ch[0] -= C5_Remotec_Offset;
                    image_ctrl.rc.ch[1] -= C5_Remotec_Offset;
                    image_ctrl.rc.ch[2] -= C5_Remotec_Offset;
                    image_ctrl.rc.ch[3] -= C5_Remotec_Offset;
                    image_ctrl.rc.ch[4] -= C5_Remotec_Offset;

//                    usart_printf("%d,%d,%d,%d\r\n",image_ctrl.rc.ch[0],image_ctrl.rc.ch[1],image_ctrl.rc.ch[2],image_ctrl.rc.ch[3]);

                    Image_Referee_KEY_AND_MOUSE_Slove();
                    Image_Key_PortHandle();
                }

            }
            break;

    }
    if(( ( RefereeFullFrame_t* )Referee_Pack)->header.DataLength +  9 < DataLength)
    {//说明缓存区内数据比一帧数据要长
        *Referee_Pack = 0xFF;//把处理过的帧破坏掉，一定要加
        (( RefereeFullFrame_t* )Referee_Pack)->frame_cmd_id = Clear_Flag;
        Referee_Pack += 9 + (( RefereeFullFrame_t* )Referee_Pack)->header.DataLength;
        Referee_Pack = Referee_Check_Frame(Referee_Pack);
    }
    else
    {
        Referee_Pack = NULL;
    }
}
/**
 * @brief 数据解包
 * @param DataLength 传入数据长度
 * @param buf_Data DMA缓存数组
 */
//void Image_Referee_UnpackRecData(uint16_t DataLength,uint8_t* buf_Data)
//{
//    static uint8_t *Referee_Pack = NULL;
//    Referee_Pack = Referee_Check_Frame(buf_Data);
//    // usart_printf("Referee_Pack:%x\r\n",Referee_Pack);
//    while (Referee_Pack != NULL)
//    {
//
//
//        switch ( ((RefereeFullFrame_t*)Referee_Pack)->frame_cmd_id )
//        {
//            case SELF_CONTROL_INTERACTIVE_ID:
//            {
//                memset((char*)&Image_referee_recv_mesg.self_control_all_data,0,sizeof(self_control_data_t));
//                memcpy((char*)&Image_referee_recv_mesg.self_control_all_data,(char*)&( (RefereeFullFrame_t*)Referee_Pack)->Data,((RefereeFullFrame_t *)Referee_Pack)->header.DataLength);
//                ++ Image_Referee_UpdateFlag.self_control_all_data;
//                // usart_printf("Self_Control_Interactive_ID\r\n");
//
//            }
//            break;
//            case KEY_AND_MOUSE_ID:
//            {
//                memcpy((char*)&Image_referee_recv_mesg.key_and_mouse,(char*)&( ( RefereeFullFrame_t* )Referee_Pack)->Data,sizeof(key_and_mouse_t));
//                ++ Image_Referee_UpdateFlag.key_and_mouse;
//                    // usart_printf("Key_And_Mouse_ID\r\n");
//
//                    }
//
//            }
//            break;
//        }
//
//}

void Image_Last_reserved(Key_and_Mouse_t_click* port)
{
    port->Last_State = port->Now_State;
}

void Image_KEY_Click_Slove(Key_and_Mouse_t_click* port)
{
    if (port->Now_State == 1 && port->Last_State == 0)
    {
        if (port->Is_Click_Once == 1)
            port->Is_Click_Once = 0;
        else
            port->Is_Click_Once = 1;

    }
    port->Is_Click_Hold = port->Now_State;
//     usart_printf("%d,%d\r\n",port->Is_Click_Hold,port->Is_Click_Once);
    port->Last_State = port->Now_State;


}

void Image_Referee_KEY_AND_MOUSE_Slove()
{
    image_ctrl.rc.Middle_Key = image_ctrl.rc.s[0];
    image_ctrl.rc.Pause_Key.Now_State = image_ctrl.rc.s[1];
    image_ctrl.rc.User_Key_L.Now_State = image_ctrl.rc.s[2];
    image_ctrl.rc.User_Key_R.Now_State = image_ctrl.rc.s[3];
    image_ctrl.rc.Rammc_Key =image_ctrl.rc.ch[4];
    image_ctrl.rc.Trigger_Key.Now_State = image_ctrl.rc.s[4];

    image_ctrl.mouse.mouse_x = image_ctrl.Key_Mouse_raw.mouse_x;
    image_ctrl.mouse.mouse_y = image_ctrl.Key_Mouse_raw.mouse_y;
    image_ctrl.mouse.mouse_z = image_ctrl.Key_Mouse_raw.mouse_z;
    image_ctrl.mouse.left_button_down.Now_State = image_ctrl.Key_Mouse_raw.left_button_down;
    image_ctrl.mouse.right_button_down.Now_State = image_ctrl.Key_Mouse_raw.right_button_down;
    image_ctrl.mouse.middle_button_down.Now_State = image_ctrl.Key_Mouse_raw.middle_button_down;

    image_ctrl.key.W.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_W);
    image_ctrl.key.S.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_S) >> 1;
    image_ctrl.key.A.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_A) >> 2;
    image_ctrl.key.D.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_D) >> 3;;
    image_ctrl.key.SHIFT.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_SHIFT) >> 4;
    image_ctrl.key.CONTRL.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_CTRL) >> 5;
    image_ctrl.key.Q.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_Q) >> 6;
    image_ctrl.key.E.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_E) >> 7;
    image_ctrl.key.R.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_R) >> 8;
    image_ctrl.key.F.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_F) >> 9;
    image_ctrl.key.G.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_G) >> 10;
    image_ctrl.key.Z.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_Z) >> 11;
    image_ctrl.key.X.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_X) >> 12;
    image_ctrl.key.C.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_C) >> 13;
    image_ctrl.key.V.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_V) >> 14;
    image_ctrl.key.B.Now_State = (image_ctrl.Key_Mouse_raw.keyboard_value & KEY_B) >> 15;

}



void Image_Key_PortHandle(){
    Image_KEY_Click_Slove(&image_ctrl.mouse.left_button_down);
    Image_KEY_Click_Slove(&image_ctrl.mouse.middle_button_down);
    Image_KEY_Click_Slove(&image_ctrl.mouse.right_button_down);

    Image_KEY_Click_Slove(&image_ctrl.rc.Trigger_Key);
    Image_KEY_Click_Slove(&image_ctrl.rc.Pause_Key);
    Image_KEY_Click_Slove(&image_ctrl.rc.User_Key_L);
    Image_KEY_Click_Slove(&image_ctrl.rc.User_Key_R);

    Image_KEY_Click_Slove(&image_ctrl.key.W);
    Image_KEY_Click_Slove(&image_ctrl.key.S);
    Image_KEY_Click_Slove(&image_ctrl.key.A);
    Image_KEY_Click_Slove(&image_ctrl.key.D);
    Image_KEY_Click_Slove(&image_ctrl.key.SHIFT);
    Image_KEY_Click_Slove(&image_ctrl.key.CONTRL);
    Image_KEY_Click_Slove(&image_ctrl.key.Q);
    Image_KEY_Click_Slove(&image_ctrl.key.E);
    Image_KEY_Click_Slove(&image_ctrl.key.R);
    Image_KEY_Click_Slove(&image_ctrl.key.F);
    Image_KEY_Click_Slove(&image_ctrl.key.G);
    Image_KEY_Click_Slove(&image_ctrl.key.Z);
    Image_KEY_Click_Slove(&image_ctrl.key.X);
    Image_KEY_Click_Slove(&image_ctrl.key.C);
    Image_KEY_Click_Slove(&image_ctrl.key.V);
    Image_KEY_Click_Slove(&image_ctrl.key.B);
}

/**
 * 保护检测 如果遥控器指定保护或遥控器断连
 * @return Car.is_protect true
 */
bool C5_portSetProtect()
{
    bool is_protect;
    C5_GetNewData.live_time--;
    if (C5_GetNewData.live_time <= 0)
        is_protect = true;
    else is_protect = false;
    return is_protect;
}


Image_ctrl_t Get_Image_Ctrl()
{

    return image_ctrl;
}

const uint8_t CRC8_INIT = CRC8_INIT_NUM;
const uint8_t CRC8_TAB[256] = {
        0x00, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
        0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x01, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
        0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x03, 0x80, 0xde, 0x3c, 0x62,
        0xbe, 0xe0, 0x02, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
        0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x07,
        0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x06, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
        0x65, 0x3b, 0xd9, 0x87, 0x04, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
        0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x05, 0xe7, 0xb9,
        0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0x0f, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
        0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0x0e, 0x50,
        0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0x0c, 0x52, 0xb0, 0xee,
        0x32, 0x6c, 0x8e, 0xd0, 0x53, 0x0d, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
        0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x08, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
        0x57, 0x09, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
        0xe9, 0xb7, 0x55, 0x0b, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
        0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0x0a, 0x54, 0xd7, 0x89, 0x6b, 0x35,
};

uint16_t CRC_INIT = CRC16_INIT_NUM;
const uint16_t wCRC_Table[256] = {
        0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
        0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
        0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
        0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
        0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
        0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
        0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
        0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
        0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
        0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
        0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
        0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
        0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
        0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
        0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
        0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
        0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
        0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
        0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
        0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
        0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
        0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
        0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
        0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
        0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
        0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
        0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
        0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
        0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
        0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
        0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
        0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};


/*
** Descriptions: CRC8 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint8_t GetCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length, uint8_t uc_crc8)
{
    uint8_t uc_index;
    while (dw_length--)
    {
        uc_index = uc_crc8 ^ (*pch_message++);
        uc_crc8 = CRC8_TAB[uc_index];
    }
    return (uc_crc8);
}

/*
** Descriptions: CRC8 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t VerifyCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length)
{
    uint8_t uc_expected = 0;
    if ((pch_message == 0) || (dw_length <= 2))
        return 0;
    uc_expected = GetCRC8CheckSum(pch_message, dw_length - 1, CRC8_INIT);
    return (uc_expected == pch_message[dw_length - 1]);
}

/*
** Descriptions: append CRC8 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void AppendCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length)
{
    uint8_t uc_crc = 0;
    if ((pch_message == 0) || (dw_length <= 2))
        return;
    uc_crc = GetCRC8CheckSum((uint8_t *) pch_message, dw_length - 1, CRC8_INIT);
    pch_message[dw_length - 1] = uc_crc;
}

/*
** Descriptions: CRC16 checksum function
** Input: Data to check,Stream length, initialized checksum
** Output: CRC checksum
*/
uint16_t GetCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length, uint16_t w_crc)
{
    uint8_t ch_data;
    if (pch_message == NULL)
        return 0xFFFF;
    while (dw_length--)
    {
        ch_data = *pch_message++;
        (w_crc) = ((uint16_t) (w_crc) >> 8) ^ wCRC_Table[((uint16_t) (w_crc) ^ (uint16_t) (ch_data)) & 0x00ff];
    }
    return w_crc;
}

/*
** Descriptions: CRC16 Verify function
** Input: Data to Verify,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
uint8_t VerifyCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length)
{
    uint16_t w_expected = 0;
    if ((pch_message == NULL) || (dw_length <= 2))
    {
        return 0;
    }
    w_expected = GetCRC16CheckSum(pch_message, dw_length - 2, CRC_INIT);
    return ((w_expected & 0xff) == pch_message[dw_length - 2] &&
            ((w_expected >> 8) & 0xff) == pch_message[dw_length - 1]);
}

/*
** Descriptions: append CRC16 to the end of data
** Input: Data to CRC and append,Stream length = Data + checksum
** Output: True or False (CRC Verify Result)
*/
void AppendCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length)
{
    uint16_t w_crc = 0;
    if ((pch_message == NULL) || (dw_length <= 2))
    {
        return;
    }
    w_crc = GetCRC16CheckSum((uint8_t *) pch_message, dw_length - 2, CRC_INIT);
    pch_message[dw_length - 2] = (uint8_t) (w_crc & 0x00ff);
    pch_message[dw_length - 1] = (uint8_t) ((w_crc >> 8) & 0x00ff);
}
