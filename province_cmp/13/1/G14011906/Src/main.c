/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "led.h"
#include "key.h"
#include "lcd.h"
#include "stdio.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
extern struct keys key[4];
#define DISPLAY_PSD         0
#define DISPLAY_STA         1
_Bool   display_mode  =  DISPLAY_PSD;
u8      b1_code = '@',b2_code = '@',b3_code = '@';
u16     freq = 2000 ;
u8      duty = 10 ;
u8      init_code1 = 1,init_code2 = 2,init_code3 = 3;
void LCD_Process(void)
{
    char display_buf[20];
    if(display_mode == DISPLAY_PSD)
    {
        LCD_DisplayStringLine(Line1,(unsigned char *)"        PSD        ");
        if(b1_code == '@')
            sprintf(display_buf,"    B1:%c     ",b1_code);
        else
            sprintf(display_buf,"    B1:%d     ",b1_code);
        LCD_DisplayStringLine(Line3,(unsigned char *)display_buf);
        if(b2_code == '@')
            sprintf(display_buf,"    B2:%c     ",b2_code);
        else
            sprintf(display_buf,"    B2:%d     ",b2_code);
        LCD_DisplayStringLine(Line4,(unsigned char *)display_buf);
        if(b3_code == '@')
            sprintf(display_buf,"    B3:%c     ",b3_code);
        else
            sprintf(display_buf,"    B3:%d     ",b3_code);
        LCD_DisplayStringLine(Line5,(unsigned char *)display_buf);      
    }
    else
    {
         LCD_DisplayStringLine(Line1,(unsigned char *)"        STA        ");
         sprintf(display_buf,"    F:%dHz   ",freq);   
         LCD_DisplayStringLine(Line3,(unsigned char *)display_buf);
         sprintf(display_buf,"    D:%d%%   ",duty);   
         LCD_DisplayStringLine(Line4,(unsigned char *)display_buf);
    }
}
u32 ledTick = 0;
u8 key_num = 0;
_Bool led_flag = 0;
void KEY_Process(void)
{
    if( key[0].key_ok == 1 )
    {
       if(display_mode == DISPLAY_PSD)
       {
           if( b1_code == '@' )
               b1_code = 0;
           else
               b1_code = (b1_code + 1) % 10;
       }         
        key[0].key_ok = 0;
    }
    if( key[1].key_ok == 1 )
    {
       if(display_mode == DISPLAY_PSD)
       {
           if( b2_code == '@' )
               b2_code = 0;
           else
               b2_code = (b2_code + 1) % 10;
       }         
        key[1].key_ok = 0;
    }
    if( key[2].key_ok == 1 )
    {
       if(display_mode == DISPLAY_PSD)
       {
           if( b3_code == '@' )
               b3_code = 0;
           else
               b3_code = (b3_code + 1) % 10;
       }         
        key[2].key_ok = 0;
    }
     if( key[3].key_ok == 1 )
     {
          if(display_mode == DISPLAY_PSD)
          {
             LCD_Clear(Black);
             if( b1_code == init_code1 && b2_code == init_code2 && b3_code == init_code3)
             {
                 
                  display_mode = DISPLAY_STA;
             }
             else
             {
                 b1_code = '@';
                 b2_code = '@';
                 b3_code = '@'; 
                 key_num++;
                 if(key_num == 3)
                 {
                     led_flag = 1;
                     ledTick = uwTick;
                     key_num = 0;
                 }
             }
         }
         key[3].key_ok = 0;
     }
}
u8 rx_buf[8];
u8 rx_dat;
u8 rx_pointer = 0;
u32 uartTick = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uartTick = uwTick;
    rx_buf[rx_pointer++] = rx_dat;
    HAL_UART_Receive_IT(&huart1,&rx_dat,1);
}

int Is_RealCode(char *rx_buf)
{
    if(((rx_buf[0] - '0')== init_code1 )&& ((rx_buf[1] - '0') == init_code2 )&& ((rx_buf[2] - '0') == init_code3))
        return 1;
    return 0;
}

int String_Check(char *str)
{
    if((str[0] >= '0' && str[0] <= '9' ) && (str[1] >= '0' && str[1] <= '9' ) && (str[2] >= '0' && str[2] <= '9' ) 
                            && (str[3] == '-')&& (str[4] >= '0' && str[4] <= '9' ) && (str[5] >= '0' && str[5] <= '9' ) && (str[6] >= '0' && str[6] <= '9' ))
        return 1;
    return 0;
}

void RxIdle_Process(void)
{
    if(uwTick - uartTick < 50) return;
    uartTick = uwTick;
    if(rx_pointer == 7)
    {
        char uart_buf[20];
        if(String_Check((char *)rx_buf) == 1)
        {
                if(Is_RealCode((char *)rx_buf) == 1)
                {
                    sprintf(uart_buf,"succesful!!code is change\r\n");
                    HAL_UART_Transmit(&huart1, (u8*)uart_buf, strlen(uart_buf),  50); 
                    init_code1 = rx_buf[4] - '0';
                    init_code2 = rx_buf[5] - '0';
                    init_code3 = rx_buf[6] - '0';
                                
                }
                else
                {
                     sprintf(uart_buf,"error3\r\n");
                     HAL_UART_Transmit(&huart1, (u8*)uart_buf, strlen(uart_buf),  50);
                }
        }
        else
        {
            sprintf(uart_buf,"error1\r\n");
            HAL_UART_Transmit(&huart1, (u8*)uart_buf, strlen(uart_buf),  50);
        }
    }
    rx_pointer = 0;
    memset(rx_buf,'\0',sizeof(rx_buf));
}

void PWM_Process(void)
{
    if(display_mode == DISPLAY_PSD)
    {
        TIM2->ARR  = 1000 - 1;
        TIM2->CCR1 = 500;
        freq =       1000;
        duty =       50;
    }
    else if(display_mode == DISPLAY_STA)
    {
        TIM2->ARR  = 500 - 1;
        TIM2->CCR1 = 50;
        freq =       2000;
        duty =       100;
    }
}

u8 led_contrl = 0;

void LED_Process(void)
{
    if(uwTick - ledTick < 100) return;
    ledTick = uwTick;
    if(led_flag == 1)
        led_contrl ^= 0x02;
    else
        led_contrl &= ~0x02;
    LED_Contrl(led_contrl);
}

u16 display_time = 0;
u16 led_time = 0;
void SysTick_Handler(void)
{
  HAL_IncTick();
  if(display_mode == DISPLAY_STA)
  {
      led_contrl |= 0x01;
      if( ++display_time >= 5000)
      {
           led_contrl &= ~0x01;
           LCD_Clear(Black);
           display_mode = DISPLAY_PSD;
           b1_code = '@';
           b2_code = '@';
           b3_code = '@'; 
           display_time = 0;
      }
  }
  if(led_flag == 1)
  {
      if(++led_time >= 5000)
      {
        led_flag = 0;
        led_time = 0;
      }
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
    HAL_UART_Receive_IT(&huart1,&rx_dat,1);
    TIM2->CCR1 = 500;
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      LCD_Process();
      KEY_Process();
      PWM_Process();
      LED_Process();
      RxIdle_Process();
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV2;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the peripherals clocks
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
