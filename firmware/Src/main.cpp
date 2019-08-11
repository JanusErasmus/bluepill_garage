/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
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

#include <time.h>

#include "Utils/cli.h"
#include "Utils/terminal_serial.h"
#include "stm32f1xx_hal.h"

#include "Utils/terminal.h"
#include "Utils/utils.h"
#include "usb_device.h"
#include "interface_nrf24.h"
#include "rolling_door.h"
#include "garage_light.h"

uint8_t netAddress[] = {0x05, 0x44, 0x55};
#define payload_length 16

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;
SPI_HandleTypeDef hspi1;
ADC_HandleTypeDef hadc1;
TIM_HandleTypeDef htim2;
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
extern "C" {
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_RTC_Init(void);
static void MX_ADC1_Init(void);
}

/* Private function prototypes -----------------------------------------------*/
typedef struct {
	uint32_t timestamp;		//4
	uint8_t inputs;			//1
	uint8_t outputs;		//1
	uint16_t voltages[4];	//8
	uint16_t temperature;	//2
}__attribute__((packed, aligned(4))) nodeData_s;

void sampleLeft(uint8_t *states)
{
	int openValue = HAL_GPIO_ReadPin(LEFT_DOOR_IN_OPEN_Port, LEFT_DOOR_IN_OPEN_Pin);
	int closeValue = HAL_GPIO_ReadPin(LEFT_DOOR_IN_CLOSE_Port, LEFT_DOOR_IN_CLOSE_Pin);

	*states = openValue | (closeValue << 1);
}

void sampleRight(uint8_t *states)
{
	int openValue = HAL_GPIO_ReadPin(RIGHT_DOOR_IN_OPEN_Port, RIGHT_DOOR_IN_OPEN_Pin);
	int closeValue = HAL_GPIO_ReadPin(RIGHT_DOOR_IN_CLOSE_Port, RIGHT_DOOR_IN_CLOSE_Pin);

	*states = openValue | (closeValue << 1);
}

void setLight(bool state)
{

	GPIO_PinState pinState = GPIO_PIN_SET;
	if(state)
		pinState = GPIO_PIN_RESET;

	HAL_GPIO_WritePin(LIGHT_OUT_Port, LIGHT_OUT_Pin, pinState);
}

RollingDoor leftDoor("L", sampleLeft);
RollingDoor rightDoor("R", sampleRight);
GarageLight light(setLight);

uint32_t getADCstep()
{
	uint32_t adc = 0;
	ADC_ChannelConfTypeDef sConfig;
	sConfig.Channel = ADC_CHANNEL_VREFINT;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
	{
		adc = HAL_ADC_GetValue(&hadc1);
		//printf("REF	: %d\n", adc);
	}
	HAL_ADC_Stop(&hadc1);

	//this amount of steps measure 1.2V
	uint32_t step = 1200000000 / adc;
	printf("step %d\n", (int)step);
	return step;
}

uint32_t sampleTemperature()
{
	uint32_t adc = 0;

	ADC_ChannelConfTypeDef sConfig;
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

	HAL_ADC_Start(&hadc1);
	if(HAL_ADC_PollForConversion(&hadc1, 1000) == HAL_OK)
	{
		adc = HAL_ADC_GetValue(&hadc1);
		//printf("ADC: %d\n", adc);
	}
	HAL_ADC_Stop(&hadc1);


	return adc;
}

uint32_t getTemperature()
{
	uint32_t sum = 0;

	HAL_ADCEx_Calibration_Start(&hadc1);

	for (int k = 0; k < 8; ++k) {
		sum += sampleTemperature();
	}

	uint32_t adc = sum >> 3;
	printf("adc: %d\n" , (int)adc);
	int voltage = adc * getADCstep();//845666; //x10^9
	//printf(" *	%d\n", voltage);
	voltage = 1.43e9 - voltage;
	//printf(" -	%d\n", voltage);
	voltage /= 4.3e3;
	//printf(" /	%d\n", voltage);

	uint32_t temp = 25000 + voltage;
	printf("temp: %d\n", (int)temp);
	return temp;
}

void report(uint8_t *address)
{
//	printf("NOT reporting\n");
	nodeData_s pay;
	memset(&pay, 0, 16);
	pay.timestamp = HAL_GetTick();
	pay.temperature = getTemperature();

	//report gate status in voltages[0-1]
	pay.voltages[0] = leftDoor.getState();
	pay.voltages[1] = rightDoor.getState();
	printf("TX result %d\n", InterfaceNRF24::get()->transmit(address, (uint8_t*)&pay, 16));
}

void reportNow()
{
	report(netAddress);
}

bool NRFreceivedCB(int pipe, uint8_t *data, int len)
{
	printf("RCV PIPE# %d\n", (int)pipe);
	printf(" PAYLOAD: %d\n", len);
	diag_dump_buf(data, len);

	nodeData_s down;
	memcpy(&down, data, len);
	int hour = (down.timestamp >> 8) & 0xFF;
	int min = (down.timestamp) & 0xFF;
	printf("Set time %d:%d\n", hour, min);


	int month = (down.timestamp >> 16) & 0xFF;
	int day = (down.timestamp >> 24) & 0xFF;
	printf("Set date %d:%d\n", month, day);

	RTC_TimeTypeDef sTime;
	sTime.Hours = hour;
	sTime.Minutes = min;
	sTime.Seconds = 0;
	HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

	//Broadcast pipe
	if(pipe == 1)
	{
		report(netAddress);
	}

	//command to node
	if(pipe == 0)
	{
		printf("Set lights %d\n", down.outputs);

		if(down.outputs & 0x01)
		{
			printf("Setting lights\n");
		}
	}

	return false;
}

int main(void)
{
  /* MCU Configuration----------------------------------------------------------*/
  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();

 // DrivewayMotors motors(&lights, &streetGate, &houseGate);

  HAL_Delay(1000);

  MX_USB_DEVICE_Init();
  sTerminalInterface_t usb = {
		  MX_USB_DEVICE_ready,
		  MX_USB_DEVICE_transmit
    };

  terminal_serial_Init();
  sTerminalInterface_t serial = {
		  terminal_serial_ready,
		  terminal_serial_transmit
  };

  sTerminalInterface_t *interfaces[] = {
		  &serial,
		  &usb,
		  0
  };

  terminal_init((sTerminalInterface_t **)&interfaces);

  MX_SPI1_Init();
  MX_ADC1_Init();

  if(HAL_GPIO_ReadPin(NRF_ADDR0_GPIO_Port, NRF_ADDR0_Pin) == GPIO_PIN_RESET)
	  netAddress[0] |= 0x01;

  if(HAL_GPIO_ReadPin(NRF_ADDR1_GPIO_Port, NRF_ADDR1_Pin) == GPIO_PIN_RESET)
	  netAddress[0] |= 0x02;

  InterfaceNRF24::init(&hspi1, netAddress, 3);
  InterfaceNRF24::get()->setRXcb(NRFreceivedCB);

  printf("Bluepill @ %dHz\n", (int)HAL_RCC_GetSysClockFreq());
  MX_RTC_Init();

  leftDoor.setLigth(&light);
  rightDoor.setLigth(&light);

  //Run doors a few times
  for (int k = 0; k < 5; ++k)
  {
	  leftDoor.run();
	  rightDoor.run();
  }

//  report(netAddress);
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);

  /* Infinite loop */
  while (1)
  {
	  terminal_run();
	  if(leftDoor.run() || rightDoor.run())
	  {
		reportNow();
	  }

	  InterfaceNRF24::get()->run();

	  light.run();

      HAL_Delay(100);
      HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
  }

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_USB|RCC_PERIPHCLK_ADC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
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
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* RTC init function */
static void MX_RTC_Init(void)
{
  RTC_TimeTypeDef sTime;
  RTC_DateTypeDef sDate;

//  RCC->APB1ENR |= (RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);


    /**Initialize RTC Only 
    */
  hrtc.Instance = RTC;
  hrtc.State = HAL_RTC_STATE_RESET;
  hrtc.Init.AsynchPrediv = RTC_AUTO_1_SECOND;
  hrtc.Init.OutPut = RTC_OUTPUTSOURCE_NONE;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
//  RTC_TamperTypeDef tamper;
//  tamper.Tamper = RTC_TAMPER_1;
//  tamper.Trigger = RTC_TAMPERTRIGGER_HIGHLEVEL;
//  HAL_RTCEx_SetTamper(&hrtc, &tamper);
  HAL_RTCEx_DeactivateTamper(&hrtc, RTC_TAMPER_1);

    /**Initialize RTC and set the Time and Date 
    */
  if(HAL_RTCEx_BKUPRead(&hrtc, RTC_BKP_DR1) == 0x32F2)
  {
	  printf(GREEN("RTC: "));

	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
	  printf("RTC date");
	  printf(" - %04d-%02d-%02d ", 2000 + sDate.Year, sDate.Month, sDate.Date);
	  printf("%02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
  }
  else
  {
	  printf(RED("RTC: Not set\n"));
	  sTime.Hours = 0;
	  sTime.Minutes = 0;
	  sTime.Seconds = 0;

	  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
	  sDate.Year = 0;
	  sDate.Month = RTC_MONTH_JANUARY;
	  sDate.Date = 0;

	  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN) != HAL_OK)
	  {
		  _Error_Handler(__FILE__, __LINE__);
	  }

	  HAL_RTCEx_BKUPWrite(&hrtc,RTC_BKP_DR1,0x32F2);
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
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	GPIO_InitStruct.Pin = GPIO_PIN_12;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);

	/*Configure GPIO pin : LED_Pin */
	GPIO_InitStruct.Pin = LED_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : SPI1_CS_Pin */
	GPIO_InitStruct.Pin = SPI1_CS_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF_CE_Pin */
	GPIO_InitStruct.Pin = NRF_CE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(NRF_CE_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF_IRQ_Pin */
	GPIO_InitStruct.Pin = NRF_IRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(NRF_IRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF_ADDR0_Pin */
	GPIO_InitStruct.Pin = NRF_ADDR0_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(NRF_ADDR0_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : NRF_ADDR1_Pin */
	GPIO_InitStruct.Pin = NRF_ADDR1_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(NRF_ADDR1_GPIO_Port, &GPIO_InitStruct);

	/*Configure LIGHT GPIO pin : LIGHT_OUT_Pin */
	GPIO_InitStruct.Pin = LIGHT_OUT_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LIGHT_OUT_Port, &GPIO_InitStruct);
	HAL_GPIO_WritePin(LIGHT_OUT_Port, LIGHT_OUT_Pin, GPIO_PIN_SET);


	/*Configure DOOR pin :  */
	GPIO_InitStruct.Pin = LEFT_DOOR_IN_OPEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(LEFT_DOOR_IN_OPEN_Port, &GPIO_InitStruct);

	/*Configure DOOR pin :  */
	GPIO_InitStruct.Pin = LEFT_DOOR_IN_CLOSE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(LEFT_DOOR_IN_CLOSE_Port, &GPIO_InitStruct);

	/*Configure DOOR pin :  */
	GPIO_InitStruct.Pin = RIGHT_DOOR_IN_OPEN_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(RIGHT_DOOR_IN_OPEN_Port, &GPIO_InitStruct);

	/*Configure DOOR pin :  */
	GPIO_InitStruct.Pin = RIGHT_DOOR_IN_CLOSE_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(RIGHT_DOOR_IN_CLOSE_Port, &GPIO_InitStruct);

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE	;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

/* TIM2 init function */

const char *getDayName(int week_day)
{
	switch(week_day)
	{
	case RTC_WEEKDAY_MONDAY:
		return "Monday";
	case RTC_WEEKDAY_TUESDAY:
		return "Tuesday";
	case RTC_WEEKDAY_WEDNESDAY:
		return "Wednesday";
	case RTC_WEEKDAY_THURSDAY:
		return "Thursday";
	case RTC_WEEKDAY_FRIDAY:
		return "Friday";
	case RTC_WEEKDAY_SATURDAY:
		return "Saturday";
	case RTC_WEEKDAY_SUNDAY:
		return "Sunday";
	}

	return 0;
}

#ifdef __cplusplus
 extern "C" {
#endif

void nrf(uint8_t argc, char **argv)
{
	if(InterfaceNRF24::get())
	{
		uint8_t address[5];
		memcpy(address, netAddress, 5);

		if(argc > 1)
		{
			address[0] = strtoul(argv[1], 0, 16);
		}

		report(address);

	}
}

void adc(uint8_t argc, char **argv)
{
	uint32_t temp = getTemperature();
	printf("temp: %dmC\n", (int)temp);
}


void rtc_debug(uint8_t argc, char **argv)
{
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	if(argc > 5)
	{
		printf("Setting date %d\n", atoi(argv[5]));

		sDate.WeekDay = RTC_WEEKDAY_MONDAY;
		sDate.Year = atoi(argv[1]) - 2000;
		sDate.Month = atoi(argv[2]);
		sDate.Date = atoi(argv[3]);
		sTime.Hours = atoi(argv[4]);
		sTime.Minutes = atoi(argv[5]);
		sTime.Seconds = 0;

		RCC->APB1ENR |= (RCC_APB1ENR_BKPEN | RCC_APB1ENR_PWREN);
		//PWR->CR |= PWR_CR_DBP;
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	}


	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	printf("RTC date: %s %d\n", getDayName(sDate.WeekDay), (int)HAL_RTC_SecondsSinceEpoch(sDate, sTime));
	printf(" - %04d-%02d-%02d ", 2000 +sDate.Year, sDate.Month, sDate.Date);
	printf("%02d:%02d:%02d\n", sTime.Hours, sTime.Minutes, sTime.Seconds);
}

#ifdef __cplusplus
 }
#endif


/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif


/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

