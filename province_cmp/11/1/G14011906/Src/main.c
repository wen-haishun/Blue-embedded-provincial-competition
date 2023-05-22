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
#include "adc.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lcd.h"
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
extern  struct          keys key[4];
#define LCD_DATA        0
#define LCD_PARA        1
_Bool   lcd_mode  =     LCD_DATA;
float   volt_r37  =     0;
u32     volt_cnt  =     0;
u32     adc2_val  =     0;

u8      volt_max  =     30;
u8      volt_min  =     10;
u8      max_com   =     30;
u8      min_com   =     10;
extern u8      led_ctrl;
extern u8      led_level;

void ADC_Proc(void)
{
    HAL_ADC_Start(&hadc2);
    adc2_val = HAL_ADC_GetValue(&hadc2);
    volt_r37 = adc2_val / 4096.0f * 3.3f;
}

void LCD_Proc(void)
{
    char lcd_buf[20];
    if( lcd_mode  == LCD_DATA )
    {
        LCD_DisplayStringLine(Line0,(unsigned char *)"      Data          ");	
        sprintf(lcd_buf," V:%4.2fV            ",volt_r37);
        LCD_DisplayStringLine(Line2,(unsigned char *)lcd_buf);
        sprintf(lcd_buf," V:%ds               ",volt_cnt);
        LCD_DisplayStringLine(Line3,(unsigned char *)lcd_buf);
    }
    else
    {
        LCD_DisplayStringLine(Line0,(unsigned char *)"      Para          ");
        sprintf(lcd_buf," Vmax:%3.1fV               ",volt_max/10.0f);
        LCD_DisplayStringLine(Line2,(unsigned char *)lcd_buf);
        sprintf(lcd_buf," Vmin:%3.1fV               ",volt_min/10.0f);
        LCD_DisplayStringLine(Line3,(unsigned char *)lcd_buf);
    }
}

void KEY_Proc(void)
{
    if(key[0].key_ok == 1)
    {
        lcd_mode = !lcd_mode;
        if( lcd_mode == LCD_DATA)
        {
            if(volt_max >= volt_min + 10)  
            {     
                led_ctrl &= ~0x02;                
                max_com = volt_max;
                min_com = volt_min;
            }
            else
            {
                led_ctrl |= 0x02;
                volt_max = max_com;
                volt_min = min_com;
            }
        }
        key[0].key_ok = 0; 
    }
    if(key[1].key_ok == 1)
    {      
        if( lcd_mode  == LCD_PARA )
            volt_max = (volt_max + 1) % 34;
        key[1].key_ok = 0; 
    }
    if(key[2].key_ok == 1)
    {
        if( lcd_mode  == LCD_PARA )
            volt_min = (volt_min + 1) % 34;
        
        key[2].key_ok = 0; 
    }
    if(key[3].key_ok == 1)
    {
        key[3].key_ok = 0; 
    }
}

_Bool   cnt1_flag = 0;
_Bool   cnt2_flag = 0;
_Bool   cnt_ok = 0;

void CNT_Proc(void)
{
    if(volt_r37 < volt_min/10.0f)
        cnt1_flag = 1;
    if(volt_r37 >= volt_min / 10.0f && cnt1_flag == 1 && volt_r37 < volt_max/10.0f)
    {
        cnt_ok = 1;
        cnt1_flag = 0;
//        cnt2_flag = 1;
        led_ctrl |= 0x01;
        volt_cnt = 0;
    }
    if(volt_r37 >= volt_max / 10.0f )
    {
        cnt_ok = 0;
        led_ctrl &= ~0x01;
        cnt1_flag = 0;
    }
}


int Is_RealString(u8* rx_buf)
{
    if(rx_buf[0] >= '0' && rx_buf[0] <= '9' && rx_buf[1] == '.' && rx_buf[3] == ',' && rx_buf[5] == '.')
        if(rx_buf[2] >= '0' && rx_buf[2] <= '9' &&rx_buf[4] >= '0' && rx_buf[4] <= '9' &&rx_buf[6] >= '0' && rx_buf[6] <= '9' )
            return 1;
    return 0;
}



u32 uartTick = 0;
u8  rx_buf[8];
u8  rx_dat;
u8  rx_point;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uartTick = uwTick;
    rx_buf[rx_point++] = rx_dat;
    HAL_UART_Receive_IT(&huart1,&rx_dat,1);
}

void Idle_Proc(void)
{
    if(uwTick - uartTick < 50) return;
    uartTick = uwTick;
    if(rx_point > 0)
    {
        u8 uart_buf[10];
        if(rx_point == 7)
        {          
            if(Is_RealString(rx_buf) == 1)
            {
                volt_max = (rx_buf[0] - '0') * 10 + (rx_buf[2] - '0');
                volt_min = (rx_buf[4] - '0') * 10 + (rx_buf[6] - '0');
                if(volt_max >= volt_min + 10 &&  volt_max <= 33)
                {                
                    max_com = volt_max;
                    min_com = volt_min;
                    led_ctrl &= ~0x04;
                }
                else
                {
                    led_ctrl |= 0x04;
                    volt_max = max_com;
                    volt_min = min_com;
                }
            }
            else
            {
                led_ctrl |= 0x04;
                sprintf((char *)uart_buf,"error\r\n");
                HAL_UART_Transmit(&huart1,uart_buf,strlen((char *)uart_buf),50);
            }
        }
        else
        {
                led_ctrl |= 0x04;
             sprintf((char *)uart_buf,"error\r\n");
             HAL_UART_Transmit(&huart1,uart_buf,strlen((char *)uart_buf),50);
        }
    }
    rx_point = 0;
    memset(rx_buf,'\0',sizeof(rx_buf));
}

void LEV_Proc(void)
{
    if(volt_r37 < 0.5f) led_level = 10;
    else if(volt_r37 < 1.0f) led_level = 30;
    else if(volt_r37 < 1.5f) led_level = 50;
    else if(volt_r37 < 2.0f) led_level = 70;
    else if(volt_r37 < 2.5f) led_level = 90;
    else led_level = 100;
}
extern u8  led_time;
u32 ledTick = 0;
void LED_Proc(void)
{
    if(led_time < led_level)
    {
          led_ctrl |= 0x40;
          LED_Ctrl(led_ctrl);
    }        
    else if(led_time > led_level)
    {
          led_ctrl &= ~0x40;
         LED_Ctrl(led_ctrl);
    }
}


u32   cnt_cnt = 0;
void SysTick_Handler(void)
{
  HAL_IncTick();
  
  if(cnt_ok == 1)
  {
      cnt_cnt++;
      if(cnt_cnt == 1000)
      {
          volt_cnt++;
          cnt_cnt = 0;
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
  MX_ADC2_Init();
  MX_TIM4_Init();
  MX_USART1_UART_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_Base_Start_IT(&htim3);
	HAL_UART_Receive_IT(&huart1,&rx_dat,1);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    LCD_Proc(); 
    ADC_Proc();  
    KEY_Proc();   
    CNT_Proc(); 
    //LED_Proc();      
    Idle_Proc();  
LEV_Proc();      
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12CLKSOURCE_SYSCLK;
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
