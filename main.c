/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  *
  * COPYRIGHT(c) 2016 STMicroelectronics
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

/* Includes ------------------------------------------------------------------*/


/* USER CODE BEGIN Includes */
#include "include.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_tx;
uint16_t TIM3Interrupt=0;
uint16_t TIM2Interrupt=0;

uint8_t gBlueToothSetFlag=0;

uint8_t PER='s';

int16_t  sampleNum=0;
#if FORMAT == BIT16
uint8_t  preface[512]={0x45,0x47,0x30,0x30,0x30,0x31,0x00,0x4d,0x00,0x00,0x00,0x00,0x45,0x43,0x47,0x01,
    0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x6f,0x6f,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x32,0x30,0x35,0x35,0x34,0x34,0x35,0x34,0x34,0x38,0x32,0x34,0x35,0x35,0x36,
    0x36,0x34,0x36,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x30,
    0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x30,0x20,
    0x20,0x32,0x30,0x31,0x32,0x2f,0x30,0x31,0x2f,0x30,0x31,0x20,0x20,0x20,0x20,0x30,
    0x30,0x3a,0x30,0x30,0x3a,0x30,0x30,0x20,0x20,0x2b,0x38,0x36,0x30,0x30,0x30,0x30,
    0x30,0x30,0x30,0x30,0x30,0x30,0x30,0xff,0x00,0x00,0x00,0x05,0x00,0x00,0x00,0x01,
    0x00,0x00,0x01,0x00,0x00,0x00,0xe8,0x03,0x8c,0x00,0x28,0x00,0x02,0x03,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0xaa,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x76,0x6c,0x26,0x6e,0x2c,0xc1,0xb4,0xc1,0x22,0xc1,0x14,0xc1,0xee,0xc1,0x34,0xc2,
    0x76,0x6c,0x24,0x6e,0x2c,0xc1,0xb4,0xc1,0x22,0xc1,0x14,0xc1,0xec,0xc1,0x34,0xc2,
    0x76,0x6c,0x24,0x6e,0x2c,0xc1,0xb2,0xc1,0x22,0xc1,0x14,0xc1,0xec,0xc1,0x34,0xc2,
    0x76,0x6c,0x24,0x6e,0x2e,0xc1,0xb4,0xc1,0x24,0xc1,0x14,0xc1,0xec,0xc1,0x36,0xc2,
    0x74,0x6c,0x24,0x6e,0x30,0xc1,0xb6,0xc1,0x24,0xc1,0x16,0xc1,0xee,0xc1,0x36,0xc2,
    0x76,0x6c,0x26,0x6e,0x34,0xc1,0xb8,0xc1,0x28,0xc1,0x18,0xc1,0xf2,0xc1,0x38,0xc2,
    0x78,0x6c,0x24,0x6e,0x34,0xc1,0xba,0xc1,0x28,0xc1,0x1a,0xc1,0xf4,0xc1,0x3c,0xc2,
    0x78,0x6c,0x26,0x6e,0x32,0xc1,0xb8,0xc1,0x28,0xc1,0x18,0xc1,0xf2,0xc1,0x3a,0xc2,
    0x76,0x6c,0x24,0x6e,0x32,0xc1,0xb8,0xc1,0x26,0xc1,0x18,0xc1,0xf2,0xc1,0x3a,0xc2,
    0x74,0x6c,0x24,0x6e,0x32,0xc1,0xb6,0xc1,0x24,0xc1,0x18,0xc1,0xf0,0xc1,0x3a,0xc2,
    0x76,0x6c,0x24,0x6e,0x30,0xc1,0xb6,0xc1,0x22,0xc1,0x16,0xc1,0xee,0xc1,0x36,0xc2,
    0x76,0x6c,0x24,0x6e,0x2c,0xc1,0xb2,0xc1,0x20,0xc1,0x12,0xc1,0xec,0xc1,0x32,0xc2,
    0x76,0x6c,0x24,0x6e,0x2e,0xc1,0xb2,0xc1,0x22,0xc1,0x14,0xc1,0xec,0xc1,0x32,0xc2,
    0x76,0x6c,0x24,0x6e,0x30,0xc1,0xb2,0xc1,0x22,0xc1,0x14,0xc1,0xee,0xc1,0x36,0xc2,
    0x76,0x6c,0x24,0x6e,0x32,0xc1,0xb6,0xc1,0x24,0xc1,0x16,0xc1,0xf0,0xc1,0x36,0xc2,
    0x76,0x6c,0x24,0x6e,0x34,0xc1,0xb8,0xc1,0x26,0xc1,0x18,0xc1,0xf2,0xc1,0x3a,0xc2,
    
                    };
#endif
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
static void MX_IWDG_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART1_UART_Init(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */
  uint16_t i;
  volatile uint8_t adcValue = 0;
	volatile uint32_t eAdc,voltage;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* Configure the system clock */
  SystemClock_Config();

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
	if(((KEY_COM->IDR)& KEY_PIN)!=KEY_PIN)
	{
		gBlueToothSetFlag = 1;
		LEDGreen_SET;
	}
	
  MX_DMA_Init();
  MX_ADC_Init();
  MX_IWDG_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
	BuleTooth_Init();       //蓝牙初始化
	ADC_Enable(&hadc);      //使能adc
  HAL_ADC_Start(&hadc);
  //SET_BS_Key;  //open at mode 
  /* USER CODE BEGIN 2 */
  init_ADS1x9x3 ();
  set_ADS1x9x_Start_Pin(HIGH); //for rdata指令	 
  #if FORMAT == BIT16
  for(i=0;i<512;i++)
    HAL_UART_Transmit(&huart1,&preface[i],sizeof(preface[i]),30);
  #endif
// 	while(1)
// 	{
// 		HAL_UART_Transmit(&huart1,&PER,sizeof(PER),30);
// 	}
	//Start_Sample();						//发送采集命令
	//Double_Chip_Data_get();
  /* USER CODE END 2 */
    __HAL_UART_ENABLE_IT(&huart1, UART_IT_RXNE);
 	RSerChar = 'G';					//该句强制赋值使采集信息会一直发送
	while(RSerChar!='G');
    ADS1298_DetectBase();
	HAL_TIM_Base_Start(&htim2);
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim3);
	HAL_TIM_Base_Start_IT(&htim3);
	TIM2Interrupt = 0;
	TIM3Interrupt = 0;
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
   
	
			if(TIM2Interrupt==1)
			{
				//HAL_TIM_Base_Stop_IT(&htim3);
					TIM2Interrupt = 0;
  /* USER CODE END WHILE */
  					ADS1298_Sample();
          eAdc = HAL_ADC_GetValue(&hadc);
          voltage = eAdc * 792 +14600;
					adcValue = (uint8_t)(voltage/100000);
				
					//HAL_UART_Transmit_DMA(&huart1,ADS1298_lsData,ADS1298_LASTDATA_LWNGTH);
					//HAL_UART_Transmit_DMA(&huart1,&adcValue,1);
					getLoffP(loffP);         //采集P导联信息
					getLoffN(loffN);         //采集N导联信息
					sampleNum++;
  /* USER CODE BEGIN 3 */
				//HAL_TIM_Base_Start_IT(&htim3);
			}
			if(TIM3Interrupt==1)    //一秒中断
			{
			 //工作状态灯闪烁
    			if(adcValue>18)
    				if(LEDGreen_Is)
    					LEDGreen_CLEAR;
    				else
    					LEDGreen_SET;
    			else
    				if(LEDRed_Is)
    					LEDRed_CLEAR;
    				else
    					LEDRed_SET;
				sampleNum = 0;
				TIM3Interrupt =0;
				while(RSerChar!='G');	//收到G才开始采集数据
 				//while(1);		//此句工作，则只发送一秒的数据
			}
  }
	
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

  __PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI
                              |RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);

  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_SYSCLK;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

}

/* ADC init function */
void MX_ADC_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION12b;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc.Init.EOCSelection = EOC_SEQ_CONV;
  hadc.Init.LowPowerAutoWait = ADC_AUTOWAIT_DISABLE;
  hadc.Init.LowPowerAutoPowerOff = ADC_AUTOPOWEROFF_DISABLE;
  hadc.Init.ChannelsBank = ADC_CHANNELS_BANK_A;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.NbrOfConversion = 1;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  HAL_ADC_Init(&hadc);

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_4CYCLES;
  HAL_ADC_ConfigChannel(&hadc, &sConfig);

}

/* IWDG init function */
void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_128;
  hiwdg.Init.Reload = 4095;
  HAL_IWDG_Init(&hiwdg);

}

/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLED;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLED;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}

/* TIM2 init function */
void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 320;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 400;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim2);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig);

}

/* TIM3 init function */
void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 32000;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 500;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  HAL_TIM_Base_Init(&htim3);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

/* USART1 init function */
void MX_USART1_UART_Init(void)
{

  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_8;
  HAL_UART_Init(&huart1);

}

/** 
  * Enable DMA controller clock
  */
void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __GPIOC_CLK_ENABLE();
  __GPIOH_CLK_ENABLE();
  __GPIOA_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();

	/*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
	
  /*Configure GPIO pin : PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB10 PB12 
                           PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|GPIO_PIN_12 
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB11 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PB5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
