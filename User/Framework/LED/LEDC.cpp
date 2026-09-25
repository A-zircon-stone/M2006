//
// Created by hx on 2024/10/3.
//

#include "LEDC.h"
/* Private includes ----------------------------------------------------------*/
#include "ledio.h"
/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
LedC led1(GPIOF, GPIO_PIN_0, LED_ON);   //LED_1 PF0
LedC led2(GPIOF, GPIO_PIN_1, LED_ON);   //LED_2 PF1
LedC led3(GPIOB, GPIO_PIN_14, LED_ON);   //LED_3 PB12
LedC led4(GPIOB, GPIO_PIN_15, LED_ON);   //LED_4 PB13
LedC led5(GPIOB, GPIO_PIN_13, LED_ON);   //LED_5 PB14
LedC led6(GPIOB, GPIO_PIN_12, LED_ON);   //LED_6 PB15

/* Private macro -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/

void (*setLedLevel)(LedC led, uint8_t level);

/* Private user code ---------------------------------------------------------*/
void LED_SetLedOn(LedC &led) {
    led.state = LED_ON;
    setLedLevel(led, led.on_level);
}

void LED_SetLedOff(LedC &led) {
    led.state = LED_OFF;
    setLedLevel(led, !led.on_level);
}

void LED_SetLedToggle(LedC &led) {
    led.state = LED_TOGGLE;
    setLedLevel(led, 3);
}

/**
  * @brief  设置led的状态
  * @param  status: LED_OFF, LED_ON, LED_TOGGLE
  * @retval None
  */
void LedC::setStatus(LED_State_e status) {
    this->state = status;
    switch (status) {
        case LED_ON:
            LED_SetLedOn(*this);
            break;
        case LED_OFF:
            LED_SetLedOff(*this);
            break;
        case LED_TOGGLE:
            LED_SetLedToggle(*this);
            break;
    }
}
void LedC::setStatus(int status) {
    switch (status) {
        case 0:
            LED_SetLedOn(*this);
            break;
        case 1:
            LED_SetLedOff(*this);
            break;
        case 2:
            LED_SetLedToggle(*this);
            break;
    }
}

void LED_Init(void) {
    LEDIO_ConfigInit();
    LedC led(GPIOE, GPIO_PIN_11, LED_ON);   //LED_R PE11
    led.setStatus(LED_TOGGLE);
}

///--------物理ui接口--------///

///Led_Ui 24译码器灯在右边 为Led1.....

void UI_Init()
{
    LED_UI(3,1,1,1,1);
}

void LED_UI(int Led1, bool Led2, bool Led3, bool Led4, bool Led5) //Led_Ui 24译码器灯为Led1.....0为亮，1为灭
{
    LEDIO_ConfigInit();
    int Led1x[2];
    switch (Led1) {
        case 0:
            Led1x[0] = 0;
            Led1x[1] = 0;
            break;
        case 1:
            Led1x[0] = 0;
            Led1x[1] = 1;
            break;
        case 2:
            Led1x[0] = 1;
            Led1x[1] = 0;
            break;
        case 3:
            Led1x[0] = 1;
            Led1x[1] = 1;
            break;
    }
    led1.setStatus(Led1x[0]);
    led2.setStatus(Led1x[1]);
    led3.setStatus(Led2);
    led4.setStatus(Led3);
    led5.setStatus(Led4);
    led6.setStatus(Led5);
}

void UI_Fric(LED_State_e status)  ///摩擦轮，最左边的LED灯
{
    led6.setStatus(status);
}

void UI_Protect(LED_State_e status)  ///射击模式，左边第二的LED灯
{
    led5.setStatus(status);
}

void UI_Front(LED_State_e status)  ///45度面敌，左边第三的LED灯
{
    led4.setStatus(status);
}

void UI_PC(LED_State_e status)  ///小电脑状态，左边第四的LED灯
{
    led3.setStatus(status);
}

void UI_gyroscope(int Led1)  ///小陀螺， 3是关闭
{
    int Led1x[2];
    switch (Led1) {
        case 0:
            Led1x[0] = 0;
            Led1x[1] = 0;
            break;
        case 1:
            Led1x[0] = 0;
            Led1x[1] = 1;
            break;
        case 2:
            Led1x[0] = 1;
            Led1x[1] = 0;
            break;
        case 3:
            Led1x[0] = 1;
            Led1x[1] = 1;
            break;
    }
    led1.setStatus(Led1x[0]);
    led2.setStatus(Led1x[1]);
}

#define ONE_PULSE        (7)                           //1 码
#define ZERO_PULSE       (3)                           //0 码
#define RESET_PULSE      (8)                           //80 ，复位信号
#define LED_NUMS         (30)                            //led 数量
#define LED_DATA_LEN     (24)                           //led 长度，单个需要24bits
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        //ws2812灯条需要的数组长度


extern TIM_HandleTypeDef htim8;
extern DMA_HandleTypeDef hdma_tim8_ch3;

uint16_t static RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = { 0 };


void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Stop_DMA(&htim8,TIM_CHANNEL_3);
}


void Ws2812SetRgb(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    //
    uint16_t* p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);

    for (uint16_t i = 0;i < 8;i++)
    {
        //
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;

    }
}
/*ws2812 初始化*/
void ws2812_init(uint8_t led_nums)
{
    uint16_t num_data;
    num_data = 80 + led_nums * 24;
    for(uint8_t i = 0; i < led_nums; i++)
    {
        Ws2812SetRgb(0x00, 0x00, 0x00, i);
    }
    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}
/*全亮蓝灯*/
void ws2812_blue(uint8_t led_nums)
{
    uint16_t num_data;
    num_data = 80 + led_nums * 24;
    for(uint8_t i = 0; i < led_nums; i++)
    {
        Ws2812SetRgb(0x00, 0x00, 0x22, i);
    }
    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}
/*全亮红灯*/
uint32_t RedBuff[24]={3,3,3,3,3,3,3,3,
                      7,7,7,7,7,7,7,7,
                      3,3,3,3,3,3,3,3};
void ws2812_red(uint8_t led_nums)
{
    uint16_t num_data;
    num_data = 80 + led_nums * 24;
    for(uint8_t i = 0; i < led_nums; i++)
    {
        Ws2812SetRgb(0xff, 0x00, 0x00, i);
    }
    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));

//    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(24));
}
/*全亮绿灯*/
void ws2812_green(uint8_t led_nums)
{
    uint16_t num_data;
    num_data = 80 + led_nums * 24;
    for(uint8_t i = 0; i < led_nums; i++)
    {
        Ws2812SetRgb(0x00, 0x22, 0x00, i);
    }
    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(num_data));
}

void ws2812_example(void)
{
    Ws2812SetRgb(0x00, 0x00, 0x22, 0);
    Ws2812SetRgb(0x00, 0x00, 0x22, 1);
    Ws2812SetRgb(0x00, 0x00, 0x22, 2);
    Ws2812SetRgb(0x00, 0x00, 0x22, 3);
    Ws2812SetRgb(0x00, 0x00, 0x22, 6);
    Ws2812SetRgb(0x00, 0x00, 0x22, 7);
    Ws2812SetRgb(0x00, 0x00, 0x22, 8);
    Ws2812SetRgb(0x00, 0x00, 0x22, 10);
    HAL_TIM_PWM_Start_DMA(&htim8,TIM_CHANNEL_3,(uint32_t *)RGB_buffur,(272)); //344 = 80 + 24 * LED_NUMS(8)

    HAL_Delay(500);
}
