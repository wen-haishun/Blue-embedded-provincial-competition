#include "key.h"


struct keys key[4] = { 0 };

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM4 )
    {
        key[0].key_stats = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
        key[1].key_stats = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
        key[2].key_stats = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_2);
        key[3].key_stats = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
        for(int i = 0;i < 4;i++)
        {
            switch(key[i].key_steps)
            {
                case 0:
                {
                    if(key[i].key_stats == 0)
                        key[i].key_steps = 1;
                }break;
                case 1:
                {
                    if(key[i].key_stats == 0)
                    {
                        key[i].key_ok = 1;
                        key[i].key_steps = 2;
                    }
                    else
                        key[i].key_steps = 0;
                }break;
                case 2:
                {
                    if(key[i].key_stats == 1)
                        key[i].key_steps = 0;
                }break;
            }
        }
    }
}
