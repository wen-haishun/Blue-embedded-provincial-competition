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
#define DISPLAY_DATA            0
#define DISPLAT_PARA            1
_Bool   display_mode =          DISPLAY_DATA;
u8      cnbr_num    = 0 ,       vnbr_num = 0,idle_num = 9;
u16     cnbr_pay    = 35,       vnbr_pay = 20;
_Bool   pa7_status  = 0 ;
void LCD_Process(void)
{
    char display_buf[30];
    if(display_mode == DISPLAY_DATA)
    {
        LCD_DisplayStringLine(Line1,(unsigned char *)"       Data         ");	
        sprintf(display_buf,"   CNBR:%d     ",cnbr_num);
        LCD_DisplayStringLine(Line3,(unsigned char *)display_buf);	
        sprintf(display_buf,"   VNBR:%d     ",vnbr_num);
        LCD_DisplayStringLine(Line5,(unsigned char *)display_buf);	
        sprintf(display_buf,"   IDLE:%d     ",idle_num);
        LCD_DisplayStringLine(Line7,(unsigned char *)display_buf);	
    }
    else
    {
        LCD_DisplayStringLine(Line1,(unsigned char *)"       Para         ");	
        sprintf(display_buf,"   CNBR:%4.2f     ",cnbr_pay/10.0f);
        LCD_DisplayStringLine(Line3,(unsigned char *)display_buf);	
        sprintf(display_buf,"   VNBR:%4.2f     ",vnbr_pay/10.0f);
        LCD_DisplayStringLine(Line5,(unsigned char *)display_buf);
    }
}

void KEY_Process(void)
{
    if(key[0].key_ok == 1)
    {
        display_mode = !display_mode;
        key[0].key_ok = 0;
    }
    if(key[1].key_ok == 1)
    {
        if(display_mode == DISPLAT_PARA)
        {
            cnbr_pay += 5;
            vnbr_pay += 5;
        }
        key[1].key_ok = 0;
    }
    if(key[2].key_ok == 1)
    {
        if(display_mode == DISPLAT_PARA)
        {
            cnbr_pay -= 5;
            vnbr_pay -= 5;
        }
        key[2].key_ok = 0;
    }
    if(key[3].key_ok == 1)
    {
        pa7_status = !pa7_status;
        key[3].key_ok = 0;
    }
}

void PWM_Process(void)
{
    if(pa7_status == 0)
        TIM17->CCR1 =   100;
    else
        TIM17->CCR1 = 0;
}

typedef struct
{
    u8   car_type[5];
    u8   car_num[5];
    u8   year;
    u8   month;
    u8   day;
    u8   hour;
    u8   minte;
    u8   second;
    u8   pos;
}CAR_INFO;

u8 rx_buf[23];
u8 rx_dat;
u8 rx_point;
u32 uartTick = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uartTick = uwTick;
    rx_buf[rx_point++] = rx_dat;
    HAL_UART_Receive_IT(&huart1, &rx_dat,1);
}

int IS_RealString(u8 *rx_buf)
{
}

CAR_INFO car_info[8];
void Idle_Process(void)
{
    if(uwTick - uartTick < 50) return;
    uartTick = uwTick;
    if(rx_point > 0)
    {
        if(rx_point == 22)
        {
            
        }
    } 
    rx_point = 0;
    memset(rx_buf,0,23);
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
  MX_TIM4_Init();
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
    HAL_UART_Receive_IT(&huart1, &rx_dat,1);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      LCD_Process();
      PWM_Process();
      KEY_Process();
      Idle_Process();
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
