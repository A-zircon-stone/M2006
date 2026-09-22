#include "core.h"
#include "main.h"
#include "RemoteC.h"
#include "bsp_can.hpp"


void UserInit()
{
    can_filter_init(&hcan1);
    // 当前使用 USART6 电脑串口；切回 DT7/DR16 时解除下一行注释。
    // REMOTEC_Init();
}

int main()
{
    main_Init();
    UserInit();
    OS_Init();
    while (1)
    {
    }
}



