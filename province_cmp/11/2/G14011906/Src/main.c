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
#include "lcd.h"
#include "key.h"
#include "led.h"
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
#define LCD_DATA    0
#define LCD_PARA    1
#define PWM_AUTO    0
#define PWM_MANU    1    
_Bool   lcd_mode =  LCD_DATA;
_Bool   pwm_mode =  PWM_AUTO;
u8      pa6_duty = 10,pa7_duty = 10;
float   volt_r37;
float duty;
extern struct keys key[4];
u32 adc2_val = 0;
void ADC_Proc(void)
{
    HAL_ADC_Start(&hadc2);
    adc2_val = HAL_ADC_GetValue(&hadc2);
    volt_r37 = adc2_val/4096.0f * 3.3f;
}

void LCD_Proc(void)
{
    char lcd_buf[20];
    if(lcd_mode == LCD_DATA)
    {
        LCD_DisplayStringLine(Line0,(unsigned char *)"      Data          ");
        sprintf(lcd_buf,"    V:%4.2fV      ",volt_r37);
        LCD_DisplayStringLine(Line2,(unsigned char *)lcd_buf);
        if(pwm_mode == PWM_AUTO)
            LCD_DisplayStringLine(Line4,(unsigned char *)"    Mode:AUTO          ");
        else
            LCD_DisplayStringLine(Line4,(unsigned char *)"    Mode:MANU          ");
    }
    else
    {
        LCD_DisplayStringLine(Line0,(unsigned char *)"      Para          ");
        sprintf(lcd_buf,"    PA6:%d%%      ",pa6_duty);
        LCD_DisplayStringLine(Line2,(unsigned char *)lcd_buf);
        sprintf(lcd_buf,"    PA7:%d%%      ",pa7_duty);
        LCD_DisplayStringLine(Line4,(unsigned char *)lcd_buf);
    }
}

void KEY_Proc(void)
{
    if(key[0].key_ok == 1)
    {
        LCD_Clear(Black);
        lcd_mode = !lcd_mode;
        key[0].key_ok = 0;
    }
    if(key[1].key_ok == 1)
    {
        if(lcd_mode == LCD_PARA)
        {
            pa6_duty += 10;
            if(pa6_duty > 90)
                pa6_duty = 10;
        }
        key[1].key_ok = 0;
    }
    if(key[2].key_ok == 1)
    {
        if(lcd_mode == LCD_PARA)
        {
            pa7_duty += 10;
            if(pa7_duty > 90)
                pa7_duty = 10;
        }
        key[2].key_ok = 0;
    }
    if(key[3].key_ok == 1)
    {
        pwm_mode = !pwm_mode;
        key[3].key_ok = 0;
    }
}

u16 duty6,duty7;
u32 fre6,fre7;
void PWM_Proc(void)
{
    if(pwm_mode == PWM_AUTO)
    {
        TIM16->CCR1 = (volt_r37 / 3.3f) * 10000;
        TIM17->CCR1 = (volt_r37 / 3.3f) * 5000;
    }
    else
    {
        TIM16->CCR1 = pa6_duty * 100;
        TIM17->CCR1 = pa7_duty * 50;
    }
}


u8 led_ctrl = 0;
void LED_Proc(void)
{
    if(pwm_mode == PWM_AUTO)
        led_ctrl |= 0x01;
    else
        led_ctrl &= ~0x01;      
    if(lcd_mode == LCD_DATA)   
        led_ctrl |= 0x02;
    else
        led_ctrl &= ~0x02;
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
  MX_TIM16_Init();
  MX_TIM17_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	
	LCD_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
    HAL_TIM_Base_Start_IT(&htim4);
    HAL_TIM_PWM_Start(&htim16,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
       
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      ADC_Proc();
      LCD_Proc();
      KEY_Proc();
      PWM_Proc();
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
