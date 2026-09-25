//
// Created by hx on 2025/1/14.
//

#ifndef INFANTRY_GIMBAL_CRC_H
#define INFANTRY_GIMBAL_CRC_H

#include <cstdint>
uint16_t Get_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(const uint8_t* pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t* pchMessage, uint32_t dwLength);

class crc {

};


#endif //INFANTRY_GIMBAL_CRC_H
