//
// Created by ZONE7 on 2025/4/4.
//

#ifndef CUSTOM_CONTROLLER_IMAGE_REFEREE_H
#define CUSTOM_CONTROLLER_IMAGE_REFEREE_H
// #include <cstring>
#include "stm32f4xx_hal.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "main.h"

#ifdef __cplusplus
}
#endif

#define SEND_LEN 4

#define KEY_W           ((uint16_t)1 << 0)
#define KEY_S           ((uint16_t)1 << 1)
#define KEY_A           ((uint16_t)1 << 2)
#define KEY_D           ((uint16_t)1 << 3)
#define KEY_SHIFT       ((uint16_t)1 << 4)
#define KEY_CTRL        ((uint16_t)1 << 5)
#define KEY_Q           ((uint16_t)1 << 6)
#define KEY_E           ((uint16_t)1 << 7)
#define KEY_R           ((uint16_t)1 << 8)
#define KEY_F           ((uint16_t)1 << 9)
#define KEY_G           ((uint16_t)1 << 10)
#define KEY_Z           ((uint16_t)1 << 11)
#define KEY_X           ((uint16_t)1 << 12)
#define KEY_C           ((uint16_t)1 << 13)
#define KEY_V           ((uint16_t)1 << 14)
#define KEY_B           ((uint16_t)1 << 15)

#include "DebugC.h"
#include "image_referee_struct.h"
#define CRC8_INIT_NUM    0xff
#define CRC16_INIT_NUM  0xffff

void Image_Referee_KEY_AND_MOUSE_Slove();
uint8_t* Referee_Check_Frame(uint8_t* StartByte);
void Image_Referee_UnpackRecData(uint16_t DataLength,uint8_t* buf_Data);
void C5_Image_Referee_UnpackRecData(uint16_t DataLength,uint8_t* buf_Data);
void Image_Referee_UartInit(void);

void Image_Key_PortHandle();
Image_ctrl_t Get_Image_Ctrl();
bool C5_portSetProtect();


uint8_t GetCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length, uint8_t uc_crc8);

uint8_t VerifyCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length);

void AppendCRC8CheckSum(uint8_t *pch_message, uint16_t dw_length);

uint16_t GetCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length, uint16_t w_crc);

uint8_t VerifyCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length);

void AppendCRC16CheckSum(uint8_t *pch_message, uint32_t dw_length);



#endif CUSTOM_CONTROLLER_IMAGE_REFEREE_HPP
