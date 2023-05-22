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
#include "lcd.h"
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
#define DISPLAY_MAIN 0
#define DISPLAY_PARA 1
#define PA7_LOW      0
#define PA7_DUTY     1
u8      display_mode = DISPLAY_MAIN;

u8      type_cnbr,type_vnbr = 0;
u8      type_idle = 8;
u8      pay_cnbr = 35,pay_vnbr = 20;
_Bool   pa7_mode = PA7_LOW;

void LCD_Process(void)
{
    char display_buf[20];
    if( display_mode == DISPLAY_MAIN)
    {
        LCD_DisplayStringLine(Line1 ,(unsigned char *)"       Data         ");
        
        sprintf((char *)display_buf,"   CNBR:%d         ",type_cnbr);
        LCD_DisplayStringLine(Line3 ,(u8*)display_buf);
        
        sprintf((char *)display_buf,"   VNBR:%d         ",type_vnbr);
        LCD_DisplayStringLine(Line5 ,(u8*)display_buf);
        
        sprintf((char *)display_buf,"   IDLE:%d         ",type_idle);
        LCD_DisplayStringLine(Line7 ,(u8*)display_buf);
        
         sprintf((char *)display_buf,"   f:%d         ",TIM17->CCR1);
         LCD_DisplayStringLine(Line8 ,(u8*)display_buf);
    }
    else if( display_mode == DISPLAY_PARA )
    {
        LCD_DisplayStringLine(Line1 ,(unsigned char *)"       Para         ");
        
        sprintf((char *)display_buf,"   CNBR:%4.2f         ",pay_cnbr/10.0f);
        LCD_DisplayStringLine(Line3 ,(u8*)display_buf);
        
        sprintf((char *)display_buf,"   VNBR:%4.2f         ",pay_vnbr/10.0f);
        LCD_DisplayStringLine(Line5 ,(u8*)display_buf);
    }
}

extern struct keys key[4];
void KEY_Process(void)
{
    if(key[0].key_flag == 1)
    {
        LCD_Clear(Black);
        display_mode = !display_mode;
        key[0].key_flag = 0;
    }
    if ( key[1].key_flag == 1)
    {
        if( display_mode == DISPLAY_PARA)
        {
            pay_cnbr += 5;
            pay_vnbr += 5;
        }
        key[1].key_flag = 0;
    }
    if ( key[2].key_flag == 1)
    {
        if( display_mode == DISPLAY_PARA)
        {
            pay_cnbr -= 5;
            pay_vnbr -= 5;
        }
        key[2].key_flag = 0;
    }
    if ( key[3].key_flag == 1)
    {
        pa7_mode = !pa7_mode;
        key[3].key_flag = 0;
    }
}

u8 led_contrl = 0;
void LED_Process(void)
{
    if( type_idle > 0)
        led_contrl |= 0x01;
    else
        led_contrl &= ~0x01;
    if (pa7_mode == PA7_DUTY)
        led_contrl |= 0x02;
    else
        led_contrl &= ~0x02;
    
    LED_Contrl(led_contrl);
}

void PWM_Process(void)
{
    if(pa7_mode == PA7_LOW)
    {
        TIM17->CCR1 =  100;
    }
    else
    {
        TIM17->CCR1 = 0;
    }
}

u32   uartTick = 0;
char rxdata[30];
uint8_t rxdat;
u8 rx_pointer;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    uartTick = uwTick;
    rxdata[rx_pointer++]=rxdat;
    HAL_UART_Receive_IT(&huart1,&rxdat,1); 
}
u8 now_year,now_month,now_dat,now_hour,now_minute,now_second;
u8 cfm_year,cfm_month,cfm_dat,cfm_hour,cfm_minute,cfm_second;

typedef struct
{
    u8 type[5];
    u8 code[5];
    u8 in_year;
    u8 in_month;
    u8 in_day;
    u8 in_hour;
    u8 in_minute;
    u8 in_second;
    u8 pos;
}TYPE_CAR_INFO;


_Bool Check_String(u8 *str)
{
    if(str[4] == ':' && str[9] == ':' && str[1] == 'N' && str[2] == 'B' && str[3] == 'R' && (str[0]=='C' || str[0] == 'V'))
    {
        for(int i = 10;i<21;i++)
        {
            if(str[i] < '0' || str[i] > '9')
                return 0;        
        }
        now_year  = (str[10] - '0')*10+(str[11] - '0');
        now_month = (str[12] - '0')*10+(str[13] - '0');
        now_dat   = (str[14] - '0')*10+(str[15] - '0');
        now_hour  = (str[16] - '0')*10+(str[17] - '0');
        now_minute= (str[18] - '0')*10+(str[19] - '0');
        now_second= (str[20] - '0')*10+(str[21] - '0');
        if((now_year > 99) ||(now_month > 12)||(now_dat > 31)||(now_hour > 23)||(now_minute > 59) ||(now_second>59))
        {
            return 0;
        }
        cfm_year  = (str[10] - '0')*10+(str[11] - '0');
        cfm_month = (str[12] - '0')*10+(str[13] - '0');
        cfm_dat   = (str[14] - '0')*10+(str[15] - '0');
        cfm_hour  = (str[16] - '0')*10+(str[17] - '0');
        cfm_minute= (str[18] - '0')*10+(str[19] - '0');
        cfm_second= (str[20] - '0')*10+(str[21] - '0');
        return 1;
    }
    return 0;
}

TYPE_CAR_INFO car_info[8];
int Check_Enter_Leave(u8* str) // 有则退场1，无则进场0
{
    u8 i;
    for(i = 0;i<8;i++)
    {
        if((car_info[i].code[0] == str[0])&&(car_info[i].code[1] == str[1])&&(car_info[i].code[2] == str[2])&&(car_info[i].code[3] == str[3]))
            return i + 1;
    }
    return 0;
}

int Check_Idle_Pos(void)
{
    u8 i = 0;
    for(;i<8;i++)
    {
        if(car_info[i].pos == 0)
            return i + 1;
    }
    return 0;//车位满了
}
long fee_time_sec = 0;
long fee_time_hour = 0;
u8 tmp[4];
u8 car_out_pos;
int car_in_pos;
void RxIdle_Process(void)
{
    if(uwTick - uartTick < 50) return;
    uartTick = uwTick;
    if(rx_pointer > 0)
    {
        if(rx_pointer == 22)
        {
            char temp[20];           
            if(Check_String((u8*)rxdata))
            {
                tmp[0] = rxdata[5];tmp[1] = rxdata[6];tmp[2] = rxdata[7];tmp[3] = rxdata[8];
                car_out_pos = Check_Enter_Leave((u8*)tmp);
                if(car_out_pos == 0)  // 进场
                {
                    //找空闲车位
                    car_in_pos = Check_Idle_Pos();
                    if(car_in_pos == 0)
                    {
                        sprintf(temp,"Error\r\n");
                        HAL_UART_Transmit(&huart1,(uint8_t *)temp,strlen(temp),50);  //超时时间
                        return;
                    }
                    car_info[car_in_pos - 1].type[0] = rxdata[0];car_info[car_in_pos - 1].type[1] = rxdata[1];car_info[car_in_pos - 1].type[2] = rxdata[2];car_info[car_in_pos - 1].type[3] = rxdata[3];
                    car_info[car_in_pos - 1].code[0] = rxdata[5];car_info[car_in_pos - 1].code[1] = rxdata[6];car_info[car_in_pos - 1].code[2] = rxdata[7]; car_info[car_in_pos - 1].code[3] = rxdata[8];
                    car_info[car_in_pos - 1].in_year = cfm_year;
                    car_info[car_in_pos - 1].in_month = cfm_month;
                    car_info[car_in_pos - 1].in_day = cfm_dat;
                    car_info[car_in_pos - 1].in_hour = cfm_hour;
                    car_info[car_in_pos - 1].in_minute = cfm_minute;
                    car_info[car_in_pos - 1].in_second = cfm_second;
                    car_info[car_in_pos - 1].pos = car_in_pos;
                    if(rxdata[0] =='C')
                    {
                        type_cnbr++;
                        type_idle--;
                    }
                    else if(rxdata[0] =='V')
                    {
                        type_vnbr++;
                        type_idle--;
                    }
                }   
                else              //离场
                {
                    fee_time_sec = (cfm_year - car_info[car_out_pos - 1].in_year)*365*24*60*60 
                    + (cfm_month - car_info[car_out_pos - 1].in_month)*30*24*60*60 + (cfm_dat - car_info[car_out_pos - 1].in_day)*24*60*60
                    + (cfm_hour - car_info[car_out_pos - 1].in_hour)*60*60 + (cfm_minute - car_info[car_out_pos - 1].in_minute)*60 
                    + (cfm_second - car_info[car_out_pos - 1].in_second);
                    if(fee_time_sec < 0)
                    {
                        return;
                    }
                    fee_time_hour = (fee_time_sec + 3599)/3600;
                    sprintf(temp,"%s:%s:%ld:%.2f\r\n",car_info[car_out_pos - 1].type,car_info[car_out_pos - 1].code,fee_time_hour,(double)fee_time_hour*(rxdata[0]=='C'?pay_cnbr:pay_vnbr)/10.0f);
                    HAL_UART_Transmit(&huart1,(uint8_t *)temp,strlen(temp),50);  //超时时间
                    memset(&car_info[car_out_pos - 1],0,sizeof(car_info[car_out_pos - 1]));
                     if(rxdata[0] =='C')
                    {
                        type_cnbr--;
                        type_idle++;
                    }
                    else if(rxdata[0] =='V')
                    {
                        type_vnbr--;
                        type_idle++;
                    }
                }              
            } 
            else
            {
                char temp[20];
                sprintf(temp,"Error\r\n");
                HAL_UART_Transmit(&huart1,(uint8_t *)temp,strlen(temp),50);  //超时时间           
            }
        }
        else
        {
            char temp[20];
            sprintf(temp,"Error\n\r");
            HAL_UART_Transmit(&huart1,(uint8_t *)temp,strlen(temp),50);  //超时时间
        }
         rx_pointer = 0;
         memset(rxdata,0,30);
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
  MX_TIM17_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
    //TIM17->CCR1 = 0;
    HAL_UART_Receive_IT(&huart1,&rxdat,1);
    HAL_TIM_Base_Start_IT(&htim4);
	LCD_Init();
    HAL_TIM_PWM_Start(&htim17,TIM_CHANNEL_1);
    
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	
	LCD_Clear(Black);
	LCD_SetBackColor(Black);
	LCD_SetTextColor(White);
	
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
      LCD_Process();
      KEY_Process();
      LED_Process();
      PWM_Process();
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
