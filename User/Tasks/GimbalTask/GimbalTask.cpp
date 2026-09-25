//
// Created by Andrew on 2024/8/31.
//

#include "GimbalTask.h"
#include "gimbalc.hpp"
#include "lrgb.h"
#include "LEDC.h"
#include "tim.h"


void GimbalTask(void const *argument)
{
    TickType_t current;
//    cRgb RGB(&htim5);
//    RGB.init();
    LED_UI(3, 1, 1, 1, 1);
    for(;;)
    {
//        HAL_GPIO_WritePin(GPIOH,GPIO_PIN_11,GPIO_PIN_SET);
        current = xTaskGetTickCount();
//        HAL_GPIO_TogglePin(GPIOH,GPIO_PIN_11);
//        RGB.loop();
        GimbalLoop();
        PrintTest();

        vTaskDelayUntil(&current, 1 / portTICK_RATE_MS);
    }
}
