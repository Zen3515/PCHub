
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
#include "stdlib.h"
#include "string.h"
#include "math.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
char rx_buff[10] = "IDLE";
char tx_buff[10] = "IDLE"; //SEND only temperature
osTimerId ledRunningTimer;
osTimerId fanTimerID;
osTimerId DS18B20TimerID;
osTimerId rgbRainbowTimerID;
osTimerId temperatureTimerID;
osTimerId staticColorTimerID;
osTimerId twoFadeTimerID;
osTimerId UARTListenTimerID;
//osThreadId fanThreadID;
//osThreadId DS18B20ThreadID;
//uint16_t fanSpeed = 999;//maximum 999
uint16_t fanSpeedP1[2] = {20, 600};
uint16_t fanSpeedP2[2] = {50, 800};
uint16_t fanSpeedP3[2] = {70, 900};
uint16_t fanSpeedP4[2] = {80, 999};
uint16_t const MAXFANSPEED = 999;
uint8_t const NORMALTEMPERATURE = 30;
uint8_t const HOTTEMPERATURE = 55;
uint8_t ledStep = 1;

/* Onewire structure */
TM_OneWire_t OW;

/* Array for DS18B20 ROM number */
uint8_t DS_ROM[8];

/* Temperature variable */
float temperature;

//osSemaphoreId binSemFanSpeedID;
osSemaphoreId binSemaphoreRGBStripID;

typedef enum { WAIT_FOR_COMMAND, WAIT_FOR_COLOR1, WAIT_FOR_COLOR2, WAIT_FOR_POINT1, WAIT_FOR_POINT2, WAIT_FOR_POINT3, WAIT_FOR_POINT4} CommandStage;
CommandStage cmdStage = WAIT_FOR_COMMAND;

typedef enum { RAINBOW, TEMPERATURE, STATIC_COLOR, TWOFADE} RGBMode; //Breathing is twofade with another color is 0 0 0
uint16_t rgbColour[3] = {999, 0, 0};
uint16_t rgbColour2[3] = {999, 0, 0};
RGBMode rgbMode = RAINBOW;
uint16_t decColour = 0;
uint16_t incColour = 1;
uint16_t rainBowCounter = 0;
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
/**
 * cmd Example(Maximum value) : F100, LS10 LM03
 */
//void changeFanSpeed(uint16_t speed_percent);
void reverse(char *str, int len);
int intToStr(int x, char str[], int d);
void ftoa(float n, char *res, int afterpoint);
void resetRainbowTask();
void processCommand(char* cmd){
	cmd[4] = '\0';
	if(cmd[0] == 'F'){ //F
		//changeFanSpeed((uint16_t)atoi(cmd + 1));
		cmdStage = WAIT_FOR_POINT1;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buff, 6); //080100
	} else if(cmd[0] == 'L'){ //L
		if(cmd[1] == 'S'){ //LS
			ledStep = (atoi(cmd + 2));
			if(rgbMode == RAINBOW){
				resetRainbowTask();
			}
		}else {//LM
			rgbMode = (RGBMode)(atoi(cmd + 3));
			if(rgbMode != RAINBOW){
				cmdStage = WAIT_FOR_COLOR1;
				HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buff, 9);
			}
			osTimerStop(ledRunningTimer);
			switch(rgbMode){
			case RAINBOW:
				resetRainbowTask();
				osTimerStart(rgbRainbowTimerID, 5);
				ledRunningTimer = rgbRainbowTimerID;
				break;
			case TEMPERATURE:
				osTimerStart(temperatureTimerID, 100);
				ledRunningTimer = temperatureTimerID;
				break;
			case STATIC_COLOR:
				osTimerStart(staticColorTimerID, 1000);
				ledRunningTimer = staticColorTimerID;
				break;
			case TWOFADE:
				rainBowCounter = 0;
				osTimerStart(twoFadeTimerID, 5);
				ledRunningTimer = twoFadeTimerID;
				break;
			}
		}
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	char copybuf[10];
	strncpy(copybuf, rx_buff, 10);
	switch(cmdStage){
	case WAIT_FOR_COMMAND:
		processCommand(copybuf);
		break;
	case WAIT_FOR_COLOR1:
		copybuf[10] = '\0';
		rgbColour[2] = atoi(copybuf + 6);
		copybuf[6] = '\0';
		rgbColour[1] = atoi(copybuf + 3);
		copybuf[3] = '\0';
		rgbColour[0] = atoi(copybuf);
		cmdStage = WAIT_FOR_COLOR2;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buff, 9);
		break;
	case WAIT_FOR_COLOR2:
		copybuf[10] = '\0';
		rgbColour2[2] = atoi(copybuf + 6);
		copybuf[6] = '\0';
		rgbColour2[1] = atoi(copybuf + 3);
		copybuf[3] = '\0';
		rgbColour2[0] = atoi(copybuf);
		cmdStage = WAIT_FOR_COMMAND;
		break;
	case WAIT_FOR_POINT1:
		copybuf[7] = '\0';
		fanSpeedP1[1] = (atoi(copybuf + 3)/100.0)*MAXFANSPEED;
		copybuf[3] = '\0';
		fanSpeedP1[0] = atoi(copybuf);
		cmdStage = WAIT_FOR_POINT2;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buff, 6);
		break;
	case WAIT_FOR_POINT2:
		copybuf[7] = '\0';
		fanSpeedP2[1] = (atoi(copybuf + 3)/100.0)*MAXFANSPEED;
		copybuf[3] = '\0';
		fanSpeedP2[0] = atoi(copybuf);
		cmdStage = WAIT_FOR_POINT3;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buff, 6);
		break;
	case WAIT_FOR_POINT3:
		copybuf[7] = '\0';
		fanSpeedP3[1] = (atoi(copybuf + 3)/100.0)*MAXFANSPEED;
		copybuf[3] = '\0';
		fanSpeedP3[0] = atoi(copybuf);
		cmdStage = WAIT_FOR_POINT4;
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buff, 6);
		break;
	case WAIT_FOR_POINT4:
		copybuf[7] = '\0';
		fanSpeedP4[1] = (atoi(copybuf + 3)/100.0)*MAXFANSPEED;
		copybuf[3] = '\0';
		fanSpeedP4[0] = atoi(copybuf);
		cmdStage = WAIT_FOR_COMMAND;
		break;
	}
}

void UARTListenTask(void const * argument){
//	uint8_t listenSize = 4;
	if(cmdStage == WAIT_FOR_COMMAND){
		HAL_UART_Receive_IT(&huart2, (uint8_t *)rx_buff, 4);
	}
}

// reverses a string 'str' of length 'len'
void reverse(char *str, int len)
{
    int i=0, j=len-1, temp;
    while (i<j)
    {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++; j--;
    }
}
 // Converts a given integer x to string str[].  d is the number
 // of digits required in output. If d is more than the number
 // of digits in x, then 0s are added at the beginning.
int intToStr(int x, char str[], int d)
{
    int i = 0;
    while (x)
    {
        str[i++] = (x%10) + '0';
        x = x/10;
    }

    // If number of digits required is more, then
    // add 0s at the beginning
    while (i < d)
        str[i++] = '0';

    reverse(str, i);
    str[i] = '\0';
    return i;
}
// Converts a floating point number to string.
void ftoa(float n, char *res, int afterpoint)
{
    // Extract integer part
    int ipart = (int)n;

    // Extract floating part
    float fpart = n - (float)ipart;

    // convert integer part to string
    int i = intToStr(ipart, res, 0);

    // check for display option after point
    if (afterpoint != 0)
    {
        res[i] = '.';  // add dot

        // Get the value of fraction part upto given no.
        // of points after dot. The third parameter is needed
        // to handle cases like 233.007
        fpart = fpart * pow(10, afterpoint);

        intToStr((int)fpart, res + i + 1, afterpoint);
    }
}

void DS18B20Task(void const * argument){
	/* Check if connected device is DS18B20 */
	if (TM_DS18B20_Is(DS_ROM)) {
		/* Everything is done */
		if (TM_DS18B20_AllDone(&OW)) {
			/* Read temperature from device */
			if (TM_DS18B20_Read(&OW, DS_ROM, &temperature)) {
				/* Temp read OK, CRC is OK */

				/* Start again on all sensors */
				TM_DS18B20_StartAll(&OW);

//				/* Check temperature */
//				if (temperature > 30) {
//					//TM_DISCO_LedOn(LED_RED);
//					htim4.Instance->CCR4 = 999;
//				} else {
//					//TM_DISCO_LedOff(LED_RED);
//					htim4.Instance->CCR4 = 0;
//				}
				char temp[10];
				ftoa(temperature, temp, 4);
				sprintf(tx_buff, "%s\n", temp);
				HAL_UART_Transmit(&huart2, (uint8_t*) tx_buff, strlen(tx_buff), 500);
			} else {
				/* CRC failed, hardware problems on data line */
			}
		}
	}
//	osDelay(1000);
}

void fanTask(void const * argument){
	//Blue LED
//	osSemaphoreWait(binSemFanSpeedID, 200);
	float temp = temperature;
	if(temp <= fanSpeedP1[0]){
		htim4.Instance->CCR4 = fanSpeedP1[1];
	} else if(temp <= fanSpeedP2[0]){
		htim4.Instance->CCR4 = fanSpeedP1[1] + ((fanSpeedP2[1] - fanSpeedP1[1])/(fanSpeedP2[0] - fanSpeedP1[0])*(temp - fanSpeedP1[0]));
	} else if(temp <= fanSpeedP3[0]){
		htim4.Instance->CCR4 = fanSpeedP2[1] + ((fanSpeedP3[1] - fanSpeedP2[1])/(fanSpeedP3[0] - fanSpeedP2[0])*(temp - fanSpeedP2[0]));
	} else if(temp <= fanSpeedP4[0]){
		htim4.Instance->CCR4 = fanSpeedP3[1] + ((fanSpeedP4[1] - fanSpeedP3[1])/(fanSpeedP4[0] - fanSpeedP3[0])*(temp - fanSpeedP3[0]));
	} else {
		htim4.Instance->CCR4 = fanSpeedP4[1];
	}
//	osSemaphoreRelease(binSemFanSpeedID);
//	osDelay(200);
}

void resetRainbowTask(){
	rgbColour[0] = 999;
	rgbColour[1] = 000;
	rgbColour[2] = 000;
	decColour = 0;
	incColour = 1;
	rainBowCounter = 0;
}

void rgbStrip_RainbowTask(void const * argument){
	rgbColour[decColour] -= ledStep;
	rgbColour[incColour] += ledStep;
	htim4.Instance->CCR1 = rgbColour[0];
	htim4.Instance->CCR2 = rgbColour[1];
	htim4.Instance->CCR3 = rgbColour[2];

	rainBowCounter += ledStep;
	if(rainBowCounter >= 999 - ledStep){
		rainBowCounter = 0;
		decColour = (decColour + 1) % 3;
		incColour = decColour == 2 ? 0 : decColour + 1;
	}
//	osDelay(5);
}

void rgbStrip_TemperatureTask(void const * argument){
	//TODO two color fading
	if(temperature < NORMALTEMPERATURE){
		htim4.Instance->CCR1 = rgbColour[0];
		htim4.Instance->CCR2 = rgbColour[1];
		htim4.Instance->CCR3 = rgbColour[2];
	} else if(temperature < HOTTEMPERATURE){
		float tempDiff = (temperature - NORMALTEMPERATURE) / (HOTTEMPERATURE - NORMALTEMPERATURE);
		htim4.Instance->CCR1 = rgbColour[0] + ((rgbColour2[0] - rgbColour[0])*tempDiff);
		htim4.Instance->CCR2 = rgbColour[1] + ((rgbColour2[1] - rgbColour[1])*tempDiff);
		htim4.Instance->CCR3 = rgbColour[2] + ((rgbColour2[2] - rgbColour[2])*tempDiff);
	} else {
		htim4.Instance->CCR1 = rgbColour2[0];
		htim4.Instance->CCR2 = rgbColour2[1];
		htim4.Instance->CCR3 = rgbColour2[2];
	}
//	osDelay(1000);
}

void rgbStrip_StaticTask(void const * argument){
	//TODO static color
	htim4.Instance->CCR1 = rgbColour[0];
	htim4.Instance->CCR2 = rgbColour[1];
	htim4.Instance->CCR3 = rgbColour[2];
}

void rgbStrip_TwoFadeTask(void const * argument){
	//TODO two fading color
	rainBowCounter += ledStep;

	if(rainBowCounter <= 1000){
		htim4.Instance->CCR1 = rgbColour[0] + ((rgbColour2[0] - rgbColour[0])*(rainBowCounter/1000.0));
		htim4.Instance->CCR2 = rgbColour[1] + ((rgbColour2[1] - rgbColour[1])*(rainBowCounter/1000.0));
		htim4.Instance->CCR3 = rgbColour[2] + ((rgbColour2[2] - rgbColour[2])*(rainBowCounter/1000.0));
	} else if (rainBowCounter <= 2000){
		uint16_t rbc = rainBowCounter - 1000;
		htim4.Instance->CCR1 = rgbColour2[0] + ((rgbColour[0] - rgbColour2[0])*(rbc/1000.0));
		htim4.Instance->CCR2 = rgbColour2[1] + ((rgbColour[1] - rgbColour2[1])*(rbc/1000.0));
		htim4.Instance->CCR3 = rgbColour2[2] + ((rgbColour[2] - rgbColour2[2])*(rbc/1000.0));
	} else {
		rainBowCounter = 0;
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
//	htim4.Instance->CCR1 = 999;
    //TM_DISCO_LedOn(LED_GREEN);

    /* Read ROM number */
    TM_OneWire_GetFullROM(&OW, DS_ROM);
  } else {
    /* Set LED RED */
//	htim4.Instance->CCR2 = 99;
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
//   osSemaphoreDef(BinSemaphoreFanSpeed);
//   binSemFanSpeedID = osSemaphoreCreate(osSemaphore(BinSemaphoreFanSpeed), 1);
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
  huart2.Init.BaudRate = 9600;
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
	osTimerDef(UARTListenTimer, UARTListenTask);
	UARTListenTimerID = osTimerCreate(osTimer(UARTListenTimer), osTimerPeriodic, NULL);
	osTimerStart(UARTListenTimerID, 300);

	osTimerDef(fanTimer, fanTask);
	fanTimerID = osTimerCreate(osTimer(fanTimer), osTimerPeriodic, NULL);
	osTimerStart(fanTimerID, 200);

	osTimerDef(DS18B20Timer, DS18B20Task);
	DS18B20TimerID = osTimerCreate(osTimer(DS18B20Timer), osTimerPeriodic, NULL);
	osTimerStart(DS18B20TimerID, 100);

	//LED Strip
	osTimerDef(rgbRainbowTimer, rgbStrip_RainbowTask);
	rgbRainbowTimerID = osTimerCreate(osTimer(rgbRainbowTimer), osTimerPeriodic, NULL);

	osTimerDef(temperatureTimer, rgbStrip_TemperatureTask);
	temperatureTimerID = osTimerCreate(osTimer(temperatureTimer), osTimerPeriodic, NULL);

	osTimerDef(staticColorTimer, rgbStrip_StaticTask);
	staticColorTimerID = osTimerCreate(osTimer(staticColorTimer), osTimerOnce, NULL);

	osTimerDef(twoFadeTimer, rgbStrip_TwoFadeTask);
	twoFadeTimerID = osTimerCreate(osTimer(twoFadeTimer), osTimerPeriodic, NULL);

	osTimerStart(rgbRainbowTimerID, 5);
	ledRunningTimer = rgbRainbowTimerID;

  /* Infinite loop */
  for(;;){

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
