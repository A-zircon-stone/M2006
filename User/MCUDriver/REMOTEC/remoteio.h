//
// Created by hx on 2024/9/27.
//

#ifndef FINALEXAM_REMOTEIO_H
#define FINALEXAM_REMOTEIO_H


//#include <cstdint>
#include "stm32f4xx_hal.h"

extern void REMOTEIO_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);


#endif //FINALEXAM_REMOTEIO_H
