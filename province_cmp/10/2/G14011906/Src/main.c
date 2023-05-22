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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "key.h"
#include "led.h"
#include "lcd.h"
#include "stdio.h"
#include "string.h"
#include "i2c_hal.h"
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
#define     LCD_MAIN    0
#define     LCD_SET     1
#define     UPPER       0
#define     LOWER       1
#define     NORMAL      2
extern struct keys key[4];
_Bool       lcd_mode    = LCD_MAIN;
float       r37;
u32         adc2_val    = 0;
u8          volt_status = UPPER;
u8          volt_max    = 24,volt_min = 12;
u8          led_upper   = 1,led_lower = 2;
u8          max_com     = 24,min_com  = 12;
u8          upper_com   = 1,lower_com = 2;
u8          set_choice  = 0;
u8          led_ctrl = 0; 

void LCD_Proc(void)
{
    u8 lcd_buf[20];
    if(lcd_mode == LCD_MAIN)
    {
        LCD_DisplayStringLine(Line2,(unsigned char *)"        Main       ");	
        sprintf((char *)lcd_buf," Volt:%4.2fV    ",r37);
        LCD_DisplayStringLine(Line4,(unsigned char *)lcd_buf);	
        if(volt_status == UPPER)
            LCD_DisplayStringLine(Line5,(unsigned char *)" Status:Upper       ");
        else if(volt_status == LOWER)
            LCD_DisplayStringLine(Line5,(unsigned char *)" Status:Lower       ");
        else
            LCD_DisplayStringLine(Line5,(unsigned char *)" Status:Normal       ");            
    }
    else
    {
        LCD_DisplayStringLine(Line2,(unsigned char *)"        Setting       ");	
        sprintf((char *)lcd_buf," Max Volt:%3.1fV    ",volt_max/10.0f);
        if(set_choice == 0)
        {
            LCD_SetBackColor(Green);
            LCD_DisplayStringLine(Line4,(unsigned char *)lcd_buf);	
            LCD_SetBackColor(Black);
        }
        else
            LCD_DisplayStringLine(Line4,(unsigned char *)lcd_buf);
        sprintf((char *)lcd_buf," Min Volt:%3.1fV    ",volt_min/10.0f);
        if(set_choice == 1)
        {
            LCD_SetBackColor(Green);
            LCD_DisplayStringLine(Line5,(unsigned char *)lcd_buf);
            LCD_SetBackColor(Black);
        }
        else
            LCD_DisplayStringLine(Line5,(unsigned char *)lcd_buf);
        sprintf((char *)lcd_buf," Upper:LD%d         ",led_upper);
        if(set_choice == 2)
        {
            LCD_SetBackColor(Green);
            LCD_DisplayStringLine(Line6,(unsigned char *)lcd_buf);
            LCD_SetBackColor(Black);
        }
        else
            LCD_DisplayStringLine(Line6,(unsigned char *)lcd_buf);
        sprintf((char *)lcd_buf," Lower:LD%d         ",led_lower);
        if(set_choice == 3)
        {
            LCD_SetBackColor(Green);
            LCD_DisplayStringLine(Line7,(unsigned char *)lcd_buf);
            LCD_SetBackColor(Black);
        }
        else
            LCD_DisplayStringLine(Line7,(unsigned char *)lcd_buf);
    }
}

void ADC_Proc(void)
{
    HAL_ADC_Start(&hadc2);
    adc2_val = HAL_ADC_GetValue(&hadc2);
    r37 = adc2_val / 4096.0f * 3.3f;
}

void KEY_Proc(void)
{
    if(key[0].key_ok == 1)
    {
        LCD_Clear(Black);
        led_ctrl = 0;        
        lcd_mode = !lcd_mode;
        if(lcd_mode == LCD_MAIN)
        {
            if(volt_max > volt_min && led_upper != led_lower)
            {
                max_com   = volt_max;
                min_com   = volt_min;
                upper_com = led_upper;
                lower_com = led_lower;
                
                HAL_Delay(5);
                E2prom_Write(0x11,max_com);
                E2prom_Write(0x12,min_com);
                E2prom_Write(0x13,upper_com);
                E2prom_Write(0x14,lower_com);              	
            }   
            else
            {
                volt_max  = max_com;
                volt_min  = min_com; 
                led_upper = upper_com;
                led_lower = lower_com;               
            }
        }
        key[0].key_ok = 0;
    }
    if(key[1].key_ok == 1)
    {
        set_choice = (set_choice + 1) % 4;
        key[1].key_ok = 0;
    }
    if(key[2].key_ok == 1)
    {
        if(lcd_mode == LCD_SET)
        {
            if( set_choice == 0)
            {
                if(volt_max < 33)
                    volt_max += 3 ;
            }
            else if( set_choice == 1)
            {
                if(  volt_min < 33)
                    volt_min += 3;
            }
            else if(set_choice == 2 )
            {
                if(led_upper < 8)
                    led_upper += 1;
            }
            else
            {
                if(led_lower < 8)
                    led_lower += 1;
            }
        }
        key[2].key_ok = 0;
    }
    if(key[3].key_ok == 1)
    {
        if(lcd_mode == LCD_SET)
        {
            if( set_choice == 0)
            {
                if(volt_max > 0)
                    volt_max -= 3;
            }
            else if( set_choice == 1 )
            {
                if(volt_min > 0 )
                    volt_min -= 3;
            }
            else if(set_choice == 2 )
            {
                if(led_upper > 1)
                    led_upper -= 1;
            }
            else
            {
                if(led_lower > 1)
                    led_lower -= 1;
            }
        }
        key[3].key_ok = 0;
    }
}


u32 ledTick = 0;
void LED_Proc(void)
{
    if(uwTick - ledTick < 200) return;
    ledTick = uwTick;
    if(r37 > (volt_max / 10.0f))
    {
        volt_status = 0;
        led_ctrl ^= (0x01 << (upper_com - 1));
    }
    else if(r37 < (volt_min / 10.0f))
    {
        volt_status = 1;
        led_ctrl ^= (0x01 << (lower_com - 1));
    }
    else
    {
        volt_status = 2;
        led_ctrl = 0;
    }
    LED_Ctrl(led_ctrl);
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
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	I2CInit();
    if(E2Prom_Read(0xf0) != 0xaa)
    {
        E2prom_Write(0xf0,0xaa);
        E2prom_Write(0x11,max_com);
        E2prom_Write(0x12,min_com);
        E2prom_Write(0x13,upper_com);
        E2prom_Write(0x14,lower_com);
    }
    
    max_com = volt_max  = E2Prom_Read(0x11);
    min_com = volt_min  = E2Prom_Read(0x12);
    upper_com = led_upper = E2Prom_Read(0x13);
    lower_com = led_lower = E2Prom_Read(0x14);
    
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
    HAL_TIM_Base_Start_IT(&htim4);
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      LCD_Proc();
      ADC_Proc();
      KEY_Proc();
      LED_Proc();
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC12;
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
