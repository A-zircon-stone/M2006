#include "core.h"
#include "main.h"
#include "RemoteC.h"
#include "bsp_can.hpp"


void UserInit()
{
    can_filter_init(&hcan1);
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



