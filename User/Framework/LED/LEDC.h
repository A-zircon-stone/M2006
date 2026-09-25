//
// Created by hx on 2024/10/3.
//

#ifndef FINALEXAM_LEDC_H
#define FINALEXAM_LEDC_H

/* Includes ------------------------------------------------------------------*/
#include "cstdint"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_gpio.h"
#include "main.h"
#include "stdint.h"
/* Exported defines -----------------------------------------------------------*/
#define LED_Port_IO         GPIO_TypeDef*
#define LED_Port_Pin        uint16_t

/* Exported types ------------------------------------------------------------*/
typedef enum {
    LED_OFF = 0,
    LED_ON,
    LED_TOGGLE
} LED_State_e;

/* Exported functions prototypes ---------------------------------------------*/
class LedC {
public:
    LED_State_e state = LED_OFF;
    uint8_t on_level; //点亮电平

    LED_Port_IO led_port;
    LED_Port_Pin led_pin;

    LedC(LED_Port_IO led_Io, LED_Port_Pin led_Pin, uint8_t on_Level = 0) :
            led_port(led_Io), led_pin(led_Pin), on_level(on_Level) {};

    void setStatus(LED_State_e status);
    void setStatus(int status);

private:
};
/* Exported functions  --------------------------------------------------------*/
// 指向外部一个操作底层的函数
extern void (*setLedLevel)(LedC led, uint8_t level);

void LED_Init(void);

void UI_Init();
void LED_UI(int Led1, bool Led2, bool Led3, bool Led4, bool Led5);
void UI_Fric(LED_State_e status);  ///摩擦轮，最左边的LED灯
void UI_Protect(LED_State_e status);  ///射击模式，左边第二的LED灯
void UI_Front(LED_State_e status);  ///45度面敌，左边第三的LED灯
void UI_PC(LED_State_e status);  ///未定，左边第四的LED灯
void UI_gyroscope(int Led1);  ///小陀螺， 0是关闭



void Ws2812SetRgb(uint8_t R, uint8_t G, uint8_t B, uint16_t num);
void ws2812_init(uint8_t led_nums);
void ws2812_example(void);
void ws2812_blue(uint8_t led_nums);
void ws2812_red(uint8_t led_nums);
void ws2812_green(uint8_t led_nums);
#endif //FINALEXAM_LEDC_H
