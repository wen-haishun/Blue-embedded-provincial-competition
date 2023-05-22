#include "key.h"
#include "led.h"
struct keys key[4] = {0};

u8  led_time = 0;
u8  led_ctrl = 0;
u8  led_level = 0;
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//控制亮度程序，使用之后LCD会有问题 
//    if(htim->Instance == TIM3) // 0.1ms
//    {
//        led_time = (led_time + 1) % 100;
//        if(led_time < led_level)
//        {
//              led_ctrl |= 0x40;
//              LED_Ctrl(led_ctrl);
//        }        
//        else if(led_time > led_level)
//        {
//              led_ctrl &= ~0x40;
//              LED_Ctrl(led_ctrl);
//        }
//    }
    if(htim->Instance == TIM4)
    {
        int i = 0;
        key[0].key_states = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
        key[1].key_states = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
        key[2].key_states = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
        key[3].key_states = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
        
        for(i = 0;i < 4;i++)
        {
            switch(key[i].key_steps)
            {
                case 0:
                {
                    if(key[i].key_states == 0)
                        key[i].key_steps = 1;
                }break;
                case 1:
                {
                    if(key[i].key_states == 0)
                    {
                        key[i].key_ok = 1;
                        key[i].key_steps = 2;
                    }
                    else
                        key[i].key_steps = 0;
                }break;
                case 2:
                {
                    if(key[i].key_states == 1)
                        key[i].key_steps = 0;
                }break;
            }
        }
        
    }
}


