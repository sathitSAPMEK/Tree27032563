/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2019 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "rtc.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "i2c-lcd.h"
#include <string.h>
#include <stdio.h>
#include "stm32f4xx_it.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint16_t adc1;
char msg[10];
long x,in_min,in_max,out_min,out_max;
bool check = false;
char strCheck[] = "off pump" ;
int i = 30;
int count = 2;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char time[10];
char date[10];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void I2C1_ADDR_Chk(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void interrup(void){
	
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 1, 1);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
	
	
}
void open_pump(void){
	
					HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6, 0); // turn on the Motor Relay = 0 ON 
					//check = true;
					lcd_send_cmd(0x80+8);
					lcd_send_string("open pump");
					HAL_Delay(5000);		
}
void set_alarm (void) 
{ 
	RTC_AlarmTypeDef sAlarm={0};
	
  sAlarm.AlarmTime.Hours = 0x10;
  sAlarm.AlarmTime.Minutes = 0x21;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x12;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
}
void set_time (void) 
{ 	
  RTC_TimeTypeDef sTime={0}; 
  RTC_DateTypeDef sDate={0};
	
/*----------------SET TIME----------------------*/
  sTime.Hours = 0x16;			
  sTime.Minutes = 0x59;
  sTime.Seconds = 0x30;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
	
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
/*-----------------SET DATE----------------------*/
  sDate.WeekDay = RTC_WEEKDAY_WEDNESDAY;
  sDate.Month = RTC_MONTH_MARCH;
  sDate.Date = 0x2;  
  sDate.Year = 0x20;	

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
	
  HAL_RTCEx_BKUPWrite(&hrtc, RTC_BKP_DR1, 0x32F2); // backup register 
}
void get_time(void) 
{ 
 RTC_DateTypeDef gDate; 
 RTC_TimeTypeDef gTime;
 RTC_TimeTypeDef sTime={0}; 	
/* Get the RTC current Time */ 
 HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
 
/*----------------------------SET TIME--------------------------------------*/ 
if(gTime.Hours ==8 && gTime.Minutes==00 && gTime.Seconds==01){open_pump();}
if(gTime.Hours ==17 && gTime.Minutes==00 && gTime.Seconds==01){open_pump();}


/* Get the RTC current Date */ 
	HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN); 
/* Display time Format: hh:mm:ss */ 
 sprintf((char*)time,"%02d:%02d:%02d",gTime.Hours, gTime.Minutes, gTime.Seconds); 
/* Display date Format: dd-mm-yy */ 
 sprintf((char*)date,"%02d-%02d-%2d",gDate.Date, gDate.Month, 2000 + gDate.Year); 
 
}

void display_time (void) 
{ 
 lcd_send_cmd (0x80); // send cursor to 0,0 
 lcd_send_string (time); 
 lcd_send_cmd (0xc0); // send cursor to 1,0 
 lcd_send_string (date); 

}
void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *hrtc) 
{ 
  HAL_GPIO_WritePin (GPIOA, GPIO_PIN_5, 1); // turn on the LED 
}
long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return ((x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)+20;
}

void EXTI15_10_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI15_10_IRQn 0 */
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_13))open_pump(); 
  /* USER CODE END EXTI15_10_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);

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
  MX_I2C1_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
	I2C1_ADDR_Chk();
	lcd_init ();
	set_alarm ();

	if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) != 0x32F2){ set_time(); }
	
//			lcd_send_cmd (0x01);
			lcd_send_cmd(0x80+5);
			lcd_send_string("Hello");
			HAL_Delay(3000); 

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
/*-------------------------CHECK MODE---------------------*/
		if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_5)== 0){count++;}
/*-------------------------READ ADC-----------------------*/		
		HAL_ADC_Start(&hadc1);
		adc1 = HAL_ADC_GetValue(&hadc1); //read Soil Moisture
		long percent = map(adc1,1024,0,0,100);
/*------------------------A lot of water plants-------------------*/
		if(count%2==0){
				
				lcd_send_cmd (0xc0+11); 
				lcd_send_string ("H"); 
				if(percent<40 && check == false && percent>30){
			
						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6, 0); // turn on the Motor Relay = 0 ON 
						lcd_send_cmd(0x80+8);
						check = true;
						lcd_send_string("open pump");
			
				}else{
			
						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6, 1); // turn on the Motor Relay = 1 OFF
						lcd_send_cmd(0x80+8);
						lcd_send_string("oof pump");
			} 
/*------------------------Less water plants-------------------*/						
		}else if(count%2!=0){
				
				lcd_send_cmd (0xc0+11); // send cursor to 0,0 
				lcd_send_string ("L"); 
			
				if(percent<30 && check == false){
			
						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6, 0); // turn on the Motor Relay = 0 ON 
						lcd_send_cmd(0x80+8);
						check = true;
						lcd_send_string("open pump");
			
				}else{
			
						HAL_GPIO_WritePin (GPIOA, GPIO_PIN_6, 1); // turn on the Motor Relay = 1 OFF
						lcd_send_cmd(0x80+8);
						lcd_send_string("oof pump");
			}
		}
		
		
			//lcd_send_cmd (0x01);
			lcd_send_cmd(0xc0+13);
			lcd_send_string(msg);
			check = false;
			lcd_send_cmd(0xc0+15);
			lcd_send_string("%");
			
			
		sprintf(msg, "%hu \n\r", percent);
    HAL_UART_Transmit(&huart2, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
		

		
/*-----------------Time-------------*/
		
		//set_time();     //Enabled set time and Date
 		get_time();
		display_time();
		interrup();
		HAL_Delay(500);

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
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /**Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 100;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
/**
  * @brief Check I2C1 slave addresses. Show result via UART2.
  * @param None
  * @retval None
  */
void I2C1_ADDR_Chk(void)
{
	uint8_t testAddr;
	uint8_t testData;
	char str[30];
	
	for (testAddr=0; testAddr<=127; testAddr++)
	{
		if ( HAL_OK == HAL_I2C_Master_Transmit(&hi2c1, testAddr<<1, &testData, 1, 200))
		{
			sprintf(str, "=====> Address 0x%02X is found. <=====\r\n", testAddr);
			HAL_UART_Transmit(&huart2, (uint8_t *) str, strlen(str), 300);

		}	else {
			sprintf(str, "Address 0x%02X is not found.\r\n", testAddr);
			HAL_UART_Transmit(&huart2, (uint8_t *) str, strlen(str), 300);

		}
		
		//if slave device is quite slow use HAL_Delay
		//HAL_Delay(200);
		
	}
	
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
