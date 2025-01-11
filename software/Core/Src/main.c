/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BUFFER_SIZE 480
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2S_HandleTypeDef hi2s2;
DMA_HandleTypeDef hdma_spi2_tx;

/* USER CODE BEGIN PV */
uint32_t buffer[BUFFER_SIZE] = {0};
uint8_t activeBuffHalf = 0;
uint8_t readyForData = 0;
uint8_t mute = 0;
uint8_t lastMute = 0;
uint8_t updateFreq = 0;
uint32_t freq = 48000U, lastFreq = 48000U;
uint8_t bitDepth = 16U, lastBitDepth = 16U;
uint16_t adcVal = 0;
int16_t vol = -1;
uint32_t mainSN = 344U, mainSR = 2U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2S2_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

uint8_t DAC_WRITE(uint8_t addr, uint8_t data);
void DAC_INIT();
void customSPI(uint16_t data);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_I2S_TxCpltCallback(I2S_HandleTypeDef* hi2s){
	activeBuffHalf = 0;
	updateBuffer();
}

void HAL_I2S_TxHalfCpltCallback(I2S_HandleTypeDef *hi2s){
	activeBuffHalf = 1;
	updateBuffer();
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
  HAL_Delay(1000);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2S2_Init();
  MX_USB_DEVICE_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  getInfo(buffer, BUFFER_SIZE, &activeBuffHalf);
  getMute(&mute);
  getFreqPoint(&freq, &updateFreq, &bitDepth);
  HAL_StatusTypeDef status = HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)buffer, BUFFER_SIZE);
  DAC_INIT();
  HAL_ADC_Start(&hadc1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(updateFreq){
		  HAL_I2S_DMAStop(&hi2s2);
		  HAL_I2S_DeInit(&hi2s2);
		  HAL_I2S_MspDeInit(&hi2s2);
		  if(freq != lastFreq){
			  lastFreq = freq;
			  switch(freq){
			  case 44100U:
				  mainSN = 271U;
				  mainSR = 2U;
				  break;
			  default:
				  mainSN = 344U;
				  mainSR = 2U;
				  break;
			  }
			  hi2s2.Init.AudioFreq = freq;
		  }
		  if(bitDepth != lastBitDepth){
			  lastBitDepth = bitDepth;
			  switch(bitDepth){
			  case 16U:
				  hdma_spi2_tx.Init.MemDataAlignment = (uint32_t)(2U << 13);
				  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
				  break;
			  case 24U:
				  hdma_spi2_tx.Init.MemDataAlignment = (uint32_t)(1U << 13);
				  hi2s2.Init.DataFormat = I2S_DATAFORMAT_24B;
				  break;
			  }
		  }
		  MX_I2S2_Init();
		  HAL_I2S_Transmit_DMA(&hi2s2, (uint16_t*)buffer, BUFFER_SIZE);
		  updateFreq = 0;
	  }

	  if(mute && !lastMute){
		  DAC_WRITE(18, 3);
		  lastMute = 1;
	  }
	  if(lastMute && !mute){
		  DAC_WRITE(18, 0);
		  lastMute = 0;
	  }

	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 100);
	  adcVal = HAL_ADC_GetValue(&hadc1);

	  if (vol == -1){
		  vol = adcVal >> 1;
		  DAC_WRITE(16, vol + 128);
		  DAC_WRITE(17, vol + 128);
	  }
	  else if((adcVal >> 1) != vol){
		  vol = adcVal >> 1;
		  DAC_WRITE(16, vol + 128);
		  DAC_WRITE(17, vol + 128);
	  }
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 24;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2S2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2S2_Init(void)
{

  /* USER CODE BEGIN I2S2_Init 0 */

  /* USER CODE END I2S2_Init 0 */

  /* USER CODE BEGIN I2S2_Init 1 */

  /* USER CODE END I2S2_Init 1 */
  hi2s2.Instance = SPI2;
  hi2s2.Init.Mode = I2S_MODE_MASTER_TX;
  hi2s2.Init.Standard = I2S_STANDARD_MSB;
  hi2s2.Init.DataFormat = I2S_DATAFORMAT_16B_EXTENDED;
  hi2s2.Init.MCLKOutput = I2S_MCLKOUTPUT_ENABLE;
  hi2s2.Init.AudioFreq = freq;
  hi2s2.Init.CPOL = I2S_CPOL_LOW;
  hi2s2.Init.ClockSource = I2S_CLOCK_PLL;
  hi2s2.Init.FullDuplexMode = I2S_FULLDUPLEXMODE_DISABLE;
  if (HAL_I2S_Init(&hi2s2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2S2_Init 2 */

  /* USER CODE END I2S2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream4_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SDATA_GPIO_Port, SDATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : NCS_Pin */
  GPIO_InitStruct.Pin = NCS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(NCS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : CLOCK_Pin SDATA_Pin */
  GPIO_InitStruct.Pin = CLOCK_Pin|SDATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

uint8_t DAC_WRITE(uint8_t addr, uint8_t data){

	uint16_t buf = (addr << 8) + data;

	customSPI(buf);
	HAL_Delay(10);

	return 0;
}

void DAC_INIT(){

	DAC_WRITE(16, 127);
	DAC_WRITE(17, 127);
	DAC_WRITE(18, 0);
}

void customSPI(uint16_t data){

	for(int i = 0; i < 64; i++){
	HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, 0);
	HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, 1);
	}

	HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, 0);
	HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, 0);
	for(int i = 0; i < 16; i++){
		if(data & (1 << (15-i))){
			HAL_GPIO_WritePin(SDATA_GPIO_Port, SDATA_Pin, 1);
		}
		else{
			HAL_GPIO_WritePin(SDATA_GPIO_Port, SDATA_Pin, 0);
		}
		HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, 1);
		HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, 0);
	}
	HAL_GPIO_WritePin(NCS_GPIO_Port, NCS_Pin, 1);
	HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, 1);

	for(int i = 0; i < 64; i++){
		HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, 0);
		HAL_GPIO_WritePin(CLOCK_GPIO_Port, CLOCK_Pin, 1);
	}

}


void getPllValues(uint32_t* pllsn, uint32_t* pllsr){
	*pllsn = mainSN;
	*pllsr = mainSR;
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
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
