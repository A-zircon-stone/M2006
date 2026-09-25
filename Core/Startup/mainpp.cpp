#include "core.h"
#include "main.h"
#include "RemoteC.h"
#include "bsp_can.hpp"


void UserInit()
{
    can_filter_init(&hcan1);
    // DR16 从 USART3(PC11) 接收，USART1 用于打印，USART6 用于电脑发射命令。
    REMOTEC_Init();
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



