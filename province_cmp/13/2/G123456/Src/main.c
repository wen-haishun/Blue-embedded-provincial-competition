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
#include "lcd.h"
#include "key.h"
#include "stdio.h"
#include "string.h"
#include "i2c.h"
#include "led.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
extern struct keys key[4];
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
#define DISPLAY_SHOP    0
#define DISPALY_PRICE   1
#define DISPALY_REP     2
u8      display_mode =      DISPLAY_SHOP;
u8      x_pnum = 0   ,      y_pnum = 0;
u8      x_pnum_uart = 0 ,   y_pnum_uart = 0;
u8      x_price = 10,       y_price = 10;
u8      x_store = 10,       y_store = 10;
void LCD_Process(void)
{
    char display_buf[20];
    if(display_mode == DISPLAY_SHOP)
    {
        LCD_DisplayStringLine(Line1,(unsigned char *)"        SHOP        ");	
        sprintf(display_buf,"     X:%d    ",x_pnum);
        LCD_DisplayStringLine(Line3,(unsigned char *)display_buf);
        sprintf(display_buf,"     Y:%d    ",y_pnum);
        LCD_DisplayStringLine(Line4,(unsigned char *)display_buf);
    }
    else if(display_mode == DISPALY_PRICE)
    {
        LCD_DisplayStringLine(Line1,(unsigned char *)"        PRICE        ");
        sprintf(display_buf,"     X:%3.1f    ",x_price/10.0f);
        LCD_DisplayStringLine(Line3,(unsigned char *)display_buf);
        sprintf(display_buf,"     Y:%3.1f    ",y_price/10.0f);
        LCD_DisplayStringLine(Line4,(unsigned char *)display_buf);
    }
    else
    {
        LCD_DisplayStringLine(Line1,(unsigned char *)"        REP        ");
        sprintf(display_buf,"     X:%d    ",x_store);
        LCD_DisplayStringLine(Line3,(unsigned char *)display_buf);
        sprintf(display_buf,"     Y:%d    ",y_store);
        LCD_DisplayStringLine(Line4,(unsigned char *)display_buf);
    }
}

_Bool   uart_flag = 0;
_Bool   led_flag = 0;
float   all_cost = 0;
void KEY_Process(void)
{
    if(key[0].key_com == 1)
    {
        display_mode = (display_mode+1) % 3;
        key[0].key_com = 0;
    }
    if(key[1].key_com == 1)
    {
        if(display_mode == DISPLAY_SHOP)
            x_pnum = (x_pnum + 1) % (x_store + 1);
        else if(display_mode == DISPALY_PRICE)
        {
            x_price += 1;
            if(x_price > 20)
                x_price = 10;
            EEPROM_Write(0x02,x_price);
        }
        else
        {
            x_store += 1;
            EEPROM_Write(0x00,x_store);
        }
        key[1].key_com = 0;
    }
    if(key[2].key_com == 1)
    {
        if(display_mode == DISPLAY_SHOP)
            y_pnum = (y_pnum + 1) % (y_store + 1);
        else if(display_mode == DISPALY_PRICE)
        {
            y_price += 1;
            if(y_price > 20)
                y_price = 10;
            EEPROM_Write(0x03,y_price);
        }
        else
        {
            y_store += 1;
            EEPROM_Write(0x01,y_store);
        }
        key[2].key_com = 0;
    }
    if(key[3].key_com == 1)
    {
         if(display_mode == DISPLAY_SHOP)
         {
             x_store    -= x_pnum;
             y_store    -= y_pnum;
             all_cost    = x_pnum * x_price + y_pnum * y_price;
             x_pnum_uart = x_pnum;
             y_pnum_uart = y_pnum;
             x_pnum      = 0;
             y_pnum      = 0;  
             EEPROM_Write(0x00,x_store);
             HAL_Delay(10);
             EEPROM_Write(0x01,y_store);
             uart_flag   = 1;
             led_flag    = 1;
         }
         key[3].key_com = 0;
    }
}

u8  rx_buf[2];
u8  rx_dat;
u8  rx_pointer;
u32 uartTick = 0;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uartTick = uwTick;
    rx_buf[rx_pointer++] = rx_dat;
    HAL_UART_Receive_IT(&huart1, &rx_dat, 1);
}

void RxIdle_Process(void)
{
    if(uwTick - uartTick < 50) return;
    uartTick = uwTick;
    
    if(uart_flag == 1)
    {
        char text[20];
        sprintf(text,"X:%d,Y:%d,Z:%3.1f\r\n",x_pnum_uart,y_pnum_uart,all_cost/10.0f);
        HAL_UART_Transmit(&huart1, (u8*)text, strlen(text), 50);
        uart_flag = 0;
    }
    if(rx_pointer == 1)
    {
        if(rx_buf[0] == '?')
        {
            char text[20];
            sprintf(text,"X:%3.1f,Y:%3.1f\r\n",x_price/10.0f,y_price/10.0f);
            HAL_UART_Transmit(&huart1, (u8*)text, strlen(text), 50);
        }
    }
    rx_pointer = 0;
    memset(&rx_buf,0,2);
}

u8  led_contrl = 0;
u32 ledTick = 0;
void LED_Process(void)
{
    if(uwTick - ledTick < 100) return;
    ledTick = uwTick;
    if (led_flag == 1)
        led_contrl |= 0x01;
    if(x_store == 0 && y_store == 0)
        led_contrl ^= 0x02;
    else
        led_contrl &= ~0x02;
    LED_Contrl(led_contrl);
}

u16 led_time;
void SysTick_Handler(void)
{
  HAL_IncTick();
  if (led_flag == 1)
  {
      led_time++;
      TIM2->CCR1 = 150;
      if(led_time >= 5000)
      {
          led_contrl &= ~0x01;
          TIM2->CCR1 = 25;
          led_time = 0;
          led_flag = 0;
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
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  I2CInit();
	if(EEPROM_Read(0xf0) != 0xaa)
    {
        EEPROM_Write(0xf0,0xaa);
        EEPROM_Write(0x00,x_store);
        EEPROM_Write(0x01,y_store);
        EEPROM_Write(0x02,x_price);
        EEPROM_Write(0x03,y_price);
    }
    x_store = EEPROM_Read(0x00);
    y_store = EEPROM_Read(0x01);
    x_price = EEPROM_Read(0x02);
    y_price = EEPROM_Read(0x03);
    
	LCD_Init();
    LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	HAL_UART_Receive_IT(&huart1, &rx_dat, 1);
    HAL_TIM_Base_Start_IT(&htim4);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      LCD_Process();
      KEY_Process();   
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
