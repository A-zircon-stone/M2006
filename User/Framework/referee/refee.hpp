//
// Created by hx on 2025/1/14.
//

#ifndef INFANTRY_GIMBAL_REFEE_HPP
#define INFANTRY_GIMBAL_REFEE_HPP
#include "framework_headfile.hpp"


typedef struct
{
    uint16_t heat_limit; //枪管热量限制
    uint16_t cool_rate; //枪管冷却速度
    uint16_t heat_now; //枪管当前热量
    uint16_t ammo_state; //机动枪管当前热量
    float shoot_spd; //当前射速
    uint16_t shoot_spd_limit; //弹速限制
    uint8_t our_color; //本队颜色
    uint8_t mechanism_state; //能量机关状态
}sReferee_Msg;

extern sReferee_Msg refereeMsg;
void RefereeMsg(uint32_t rx_std_id ,uint8_t *rx_buf);
class refee {

};


#endif //INFANTRY_GIMBAL_REFEE_HPP
