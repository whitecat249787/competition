#include "grab.h"
void grab(void)
{
   HAL_GPIO_TogglePin(GPIOB,GPIO_PIN_0);//所设引脚为PB0，初始为低电平，使机械爪处于松弛状态
}