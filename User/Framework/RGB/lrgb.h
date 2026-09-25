//
// Created by hx on 2024/10/4.
//

#ifndef FINALEXAM_LRGB_H
#define FINALEXAM_LRGB_H

#include <cstdint>
#include "main.h"
class cRgb
{
public:
    cRgb(TIM_HandleTypeDef *tim): _rgb_tim(tim){}

    void init();
    void loop();
private:
    TIM_HandleTypeDef *_rgb_tim;

    void set(uint32_t argb);
};


#endif //FINALEXAM_LRGB_H
