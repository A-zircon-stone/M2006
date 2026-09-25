//
// Created by hx on 2025/1/14.
//

#include "refee.hpp"
#define REFEREE_RECIRVE_1 0x405
#define REFEREE_RECIRVE_2 0x407

sReferee_Msg refereeMsg;

void RefereeMsg(uint32_t rx_std_id ,uint8_t *rx_buf)
{
    if (rx_std_id == REFEREE_RECIRVE_1)
    {
        refereeMsg.heat_limit = (uint16_t)(rx_buf[0] << 8 | rx_buf[1]);
        refereeMsg.cool_rate = (uint16_t)(rx_buf[2] << 8 | rx_buf[3]);
        refereeMsg.heat_now = (uint16_t)(rx_buf[4] << 8 | rx_buf[5]);
        refereeMsg.ammo_state = (uint16_t)(rx_buf[6]);
//        usart_printf("%d,%d,%d,%d\r\n",refereeMsg.heat_now,refereeMsg.cool_rate,refereeMsg.heat_limit,refereeMsg.ammo_state);
    }
    else if (rx_std_id == REFEREE_RECIRVE_2)
    {
//        uint32_t floatAsInt = (uint32_t)(*(rx_buf) <<24 | *(rx_buf + 1) << 16) | *(rx_buf + 2) << 8 | *(rx_buf + 3);
//        refereeMsg.shoot_spd = static_cast<float>(floatAsInt);
        memcpy(&refereeMsg.shoot_spd, rx_buf, 4);
        refereeMsg.shoot_spd_limit = (uint16_t)(rx_buf[4] << 8 | rx_buf[5]);
        refereeMsg.our_color = (uint8_t)rx_buf[6];
        refereeMsg.mechanism_state = (uint8_t)rx_buf[7];
    }

}