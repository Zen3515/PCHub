
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* Include core modules */
#include "stm32fxxx_hal.h"
/* Include my libraries here */
#include "defines.h"
//#include "tm_stm32_disco.h"
#include "tm_stm32_delay.h"
#include "tm_stm32_ds18b20.h"
#include "tm_stm32_onewire.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
uint32_t fanSpeed = 299;//maximum 999
uint32_t const MAXFANSPEED = 999;

/* Onewire structure */
TM_OneWire_t OW;

/* Array for DS18B20 ROM number */
uint8_t DS_ROM[8];

/* Temperature variable */
float temp;

osSemaphoreId binSemFanSpeedID;
osSemaphoreId binSemaphoreRGBStripID;

typedef enum { RAINBOW, TEMPERATURE, STATIC_COLOR, BREATHING } RGBMode;
uint16_t rgbColour[3] = {999, 0, 0};
RGBMode rgbMode = RAINBOW;
//typedef struct Color {uint16_t red; uint16_t green; uint16_t blue;} Color_t;
//Color_t currentColor = {999,0,0}; // start with red
//Color_t const RED = {999, 0, 0};
//Color_t const GREEN = {0, 999, 0};
//Color_t const BLUE = {0, 0, 999};
//Color_t const YELLOW = {999, 999, 0};
//Color_t const AQUA = {0, 999, 999};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM4_Init(void);
void StartDefaultTask(void const * argument);                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void fanTask(void const * argument){
	//Blue LED
	osSemaphoreWait(binSemFanSpeedID, osWaitForever);
	htim4.Instance->CCR4 = fanSpeed;
	osSemaphoreRelease(binSemFanSpeedID);
	osDelay(200);
}

void changeFanSpeed(uint32_t speed_percent){
	osSemaphoreWait(binSemFanSpeedID, osWaitForever);
	fanSpeed = speed_percent*MAXFANSPEED;
	osSemaphoreRelease(binSemFanSpeedID);
}

void rgbStripTask(void const * argument){
	osSemaphoreWait(binSemaphoreRGBStripID, osWaitForever);
	RGBMode mode = rgbMode;
	osSemaphoreRelease(binSemaphoreRGBStripID);
	switch(mode){
	case RAINBOW:
		// Choose the colours to increment and decrement.
		uint8_t decColour = 0;
		for (decColour = 0; decColour < 3; decColour += 1) {
			uint8_t incColour = decColour == 2 ? 0 : decColour + 1;

		    // cross-fade the two colours.
		    for(int i = 0; i < 999; i += 1) {
		      rgbColour[decColour] -= 1;
		      rgbColour[incColour] += 1;

//		      setColourRgb(rgbColour[0], rgbColour[1], rgbColour[2]);
		      htim4.Instance->CCR1 = rgbColour[0];
			  htim4.Instance->CCR2 = rgbColour[1];
			  htim4.Instance->CCR3 = rgbColour[2];
//		      delay(5);
			  osDelay(5);
		    }
		}
		break;
	case TEMPERATURE:
//		osSemaphoreRelease(binSemaphoreRGBStripID);
		osDelay(5);
		break;
	case BREATHING:
//		osSemaphoreRelease(binSemaphoreRGBStripID);
		osDelay(5);
		break;
	case STATIC_COLOR:
//		osSemaphoreRelease(binSemaphoreRGBStripID);
		osDelay(1000);
		break;
	default:
//		osSemaphoreRelease(binSemaphoreRGBStripID);
		osDelay(1000);
		break;
	}
}

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	//TM_RCC_InitSystem();
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_USART2_UART_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */

  /* Init ONEWIRE port on PB4 pin */
  TM_OneWire_Init(&OW, GPIOB, GPIO_PIN_4);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  /* Check if any device is connected */
  if (TM_OneWire_First(&OW)) {
	/* Set LED GREEN */
	htim4.Instance->CCR1 = 999;
    //TM_DISCO_LedOn(LED_GREEN);

    /* Read ROM number */
    TM_OneWire_GetFullROM(&OW, DS_ROM);
  } else {
    /* Set LED RED */
	htim4.Instance->CCR2 = 99;
  }

   /* Start temp conversion */
   if (TM_DS18B20_Is(DS_ROM)) {
    /* Set resolution */
    TM_DS18B20_SetResolution(&OW, DS_ROM, TM_DS18B20_Resolution_12bits);

    /* Set high and low alarms */
    TM_DS18B20_SetAlarmHighTemperature(&OW, DS_ROM, 30);
    TM_DS18B20_SetAlarmLowTemperature(&OW, DS_ROM, 10);

    /* Start conversion on all sensors */
    TM_DS18B20_StartAll(&OW);
   }

  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
   osSemaphoreDef(BinSemaphoreFanSpeed);
   binSemFanSpeedID = osSemaphoreCreate(osSemaphore(BinSemaphoreFanSpeed), 1);
   osSemaphoreDef(BinSemaphoreRGBStrip);
   binSemaphoreRGBStripID = osSemaphoreCreate(osSemaphore(BinSemaphoreRGBStrip), 1);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

  /* Start scheduler */
  osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 83;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN 5 */

	osThreadDef(fanThread, StartDefaultTask, osPriorityNormal, 0, 128);
	osThreadId fanThreadID = osThreadCreate(osThread(fanThread), NULL);

  /* Infinite loop */
  for(;;)
  {



	  /* Check if any device is connected */
	    if (TM_OneWire_First(&OW)) {
	  	/* Set LED GREEN */
	  	htim4.Instance->CCR1 = 999;
	      //TM_DISCO_LedOn(LED_GREEN);

	      /* Read ROM number */
	      TM_OneWire_GetFullROM(&OW, DS_ROM);
	    } else {
	      /* Set LED RED */
	  	htim4.Instance->CCR2 = 99;
	    }

	     /* Start temp conversion */
	     if (TM_DS18B20_Is(DS_ROM)) {
	      /* Set resolution */
	      TM_DS18B20_SetResolution(&OW, DS_ROM, TM_DS18B20_Resolution_12bits);

	      /* Set high and low alarms */
	      TM_DS18B20_SetAlarmHighTemperature(&OW, DS_ROM, 30);
	      TM_DS18B20_SetAlarmLowTemperature(&OW, DS_ROM, 10);

	      /* Start conversion on all sensors */
	      TM_DS18B20_StartAll(&OW);
	     }






	  /* Check if connected device is DS18B20 */
	  if (TM_DS18B20_Is(DS_ROM)) {
	  	/* Everything is done */
	  	if (TM_DS18B20_AllDone(&OW)) {
	  /* Read temperature from device */
	  if (TM_DS18B20_Read(&OW, DS_ROM, &temp)) {
	  	/* Temp read OK, CRC is OK */

	  	/* Start again on all sensors */
	  	TM_DS18B20_StartAll(&OW);

	  	/* Check temperature */
	  	if (temp > 30) {
	  		//TM_DISCO_LedOn(LED_RED);
	  		htim4.Instance->CCR4 = 999;
	  	} else {
	  		//TM_DISCO_LedOff(LED_RED);
	  		htim4.Instance->CCR4 = 0;
	  	}
	  } else {
	  	/* CRC failed, hardware problems on data line */
	  }
	  	}
	  }

    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
