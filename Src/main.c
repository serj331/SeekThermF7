/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
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
#include "stm32f7xx_hal.h"
#include "usb_host.h"

/* USER CODE BEGIN Includes */

#include "pallete_rain.h"
#include "usbh_seekth.h"
#include "fonts.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

DMA2D_HandleTypeDef hdma2d;

LTDC_HandleTypeDef hltdc;

SPI_HandleTypeDef hspi5;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim7;

SDRAM_HandleTypeDef hsdram1;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

extern ApplicationTypeDef Appli_state;

uint16_t __attribute__((section (".sdram_b1"))) LCDLay1FrontBuf[480 * 272];
uint16_t __attribute__((section (".sdram_b2"))) LCDLay1BackBuf[480 * 272];
uint16_t* LCDLay1FrameBuf = LCDLay1FrontBuf;

uint16_t __attribute__((section (".sdram_b3"))) FrameID3_cc[32448];
double __attribute__((section (".sdram_b3"))) GainCalibration[32448];

uint16_t FrameID1[32448];
uint16_t FrameID3[32448];
uint16_t FrameID4[32448];
uint8_t BadPixels[32448];

int CalibFrameAvgVal = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void Error_Handler(void);
static void MPU_Config(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_LTDC_Init(void);
static void MX_SPI5_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM7_Init(void);
static void MX_DMA2D_Init(void);
void MX_USB_HOST_Process(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

void initialise_monitor_handles();
void SDRAM_initSequence();

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim == &htim7) {
		MX_USB_HOST_Process();
	}
}

int compare(const void * a, const void * b) {
	int v1 = *(uint16_t*) a;
	int v2 = *(uint16_t*) b;
	return v1 - v2;
}

uint16_t GetFrameMedian(uint16_t* frame) {
	HAL_GPIO_WritePin(LCD_PENIRQ_GPIO_Port, LCD_PENIRQ_Pin, GPIO_PIN_SET);
	uint16_t sortArr[32448];
	memcpy(sortArr, frame, 64896);
	qsort(sortArr, 32448, sizeof(uint16_t), compare);
	HAL_GPIO_WritePin(LCD_PENIRQ_GPIO_Port, LCD_PENIRQ_Pin, GPIO_PIN_RESET);
	return (uint16_t) sortArr[32448 / 2];
}

void fixBadPixels() {
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t i = 0;
	uint16_t nr = 0;
	uint16_t val = 0;

	for (y = 0; y < 156; y++) {
		for (x = 0; x < 208; x++, i++) {
			if (BadPixels[i] && x < 206) {

				val = 0;
				nr = 0;

				if (y > 0 && !BadPixels[i - 208]) //top pixel
						{
					val += FrameID3[i - 208];
					++nr;
				}

				if (y < 155 && !BadPixels[i + 208]) // bottom pixel
						{
					val += FrameID3[i + 208];
					++nr;
				}

				if (x > 0 && !BadPixels[i - 1]) //Left pixel
						{
					val += FrameID3[i - 1];
					++nr;
				}

				if (x < 205 && !BadPixels[i + 1]) //Right pixel
						{
					val += FrameID3[i + 1];
					++nr;
				}

				if (nr > 0) {
					val /= nr;
					FrameID3[i] = val;
				}
			}
		}
	}
}

void removeNoise() {
	uint16_t x = 0;
	uint16_t y = 0;
	uint16_t i = 0;
	uint16_t val = 0;
	uint16_t arrColor[4];

	for (y = 0; y < 156; y++) {
		for (x = 0; x < 208; x++) {
			if (x > 0 && x < 206 && y > 0 && y < 155) {
				arrColor[0] = FrameID3[i - 208]; //top
				arrColor[1] = FrameID3[i + 208]; //bottom
				arrColor[2] = FrameID3[i - 1]; //left
				arrColor[3] = FrameID3[i + 1]; //right

				uint16_t highest = 0;
				uint16_t lowest = 30000;

				for (int i = 0; i < 4; i++) {
					if (arrColor[i] > highest)
						highest = arrColor[i];
					if (arrColor[i] < lowest)
						lowest = arrColor[i];
				}

				val = (uint16_t) (((int) arrColor[0] + arrColor[1] + arrColor[2]
						+ arrColor[3] - highest - lowest) / 2);

				if (abs(val - FrameID3[i]) > 100 && val != 0) {
					FrameID3[i] = val;
				}
			}
			i++;
		}
	}

}


/* USER CODE END 0 */

int main(void) {

	/* USER CODE BEGIN 1 */
//	initialise_monitor_handles();
	/* USER CODE END 1 */

	/* MPU Configuration----------------------------------------------------------*/
	MPU_Config();

	/* Enable I-Cache-------------------------------------------------------------*/
	SCB_EnableICache();

	/* Enable D-Cache-------------------------------------------------------------*/
	SCB_EnableDCache();

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* Configure the system clock */
	SystemClock_Config();

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_FMC_Init();

	SDRAM_initSequence();

	MX_LTDC_Init();
	MX_SPI5_Init();
	MX_TIM3_Init();
	MX_USB_HOST_Init();
	MX_TIM7_Init();
	MX_DMA2D_Init();

	/* USER CODE BEGIN 2 */

	HAL_Delay(10);

	for(int i = 0; i < 480 * 272; i++) {
		LCDLay1FrontBuf[i] = 0;
		LCDLay1BackBuf[i] = 0;
	}

	TIM3->CCR3 = 0;
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);

	for(int i = 0; i < 9; i++) {
		HAL_Delay(10);
		TIM3->CCR3 += 10;
	}

	HAL_TIM_Base_Start_IT(&htim7);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */

	while (1) {
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

		if (Appli_state == APPLICATION_READY) {
			uint16_t* rawFrame = SeekT_getFrame();
			if (rawFrame != NULL) {

				HAL_GPIO_WritePin(USB_PSO_GPIO_Port, USB_PSO_Pin, GPIO_PIN_SET);

				int frameId = ((uint8_t*) rawFrame)[20];

				switch (frameId) {
				case 1: {
					memcpy(FrameID1, rawFrame, 64896);
					CalibFrameAvgVal = GetFrameMedian(FrameID1);
					for (int i = 0; i < 32448; i++) {
						if (FrameID1[i] < 2000 || FrameID1[i] > 22000) {
							BadPixels[i] = 1;
						}
					}
				}
					break;
				case 3: {
					memcpy(FrameID3, rawFrame, 64896);
					for (int i = 0; i < 32448; i++) {
						if (FrameID3[i] > 2000 && FrameID3[i] < 30000) {
							double diff = (double) FrameID3[i] - FrameID1[i];
							FrameID3[i] =
									(uint16_t) ((diff * GainCalibration[i])
											+ CalibFrameAvgVal);
						} else {
							FrameID3[i] = 0;
							BadPixels[i] = 1;
						}
					}

					fixBadPixels();
					removeNoise();

					//find min max
					int minv = 30000;
					int maxv = 2000;
					for (int i = 0; i < 156; i++) {
						for (int j = 0; j < 206; j++) {
							int pixPos = i * 208 + j;
							int pixVal = FrameID3[pixPos];
							if (minv > pixVal) {
								minv = pixVal;
							}
							if (maxv < pixVal) {
								maxv = pixVal;
							}

						}
					}

					minv -= 0;
					maxv += 100;
					//apply pallete
					for (int i = 0; i < 156; i++) {
						for (int j = 0; j < 206; j++) {
							int pixPos = i * 208 + j;
							FrameID3_cc[pixPos] = RainPallete565[(int) ((double) ((FrameID3[pixPos] - minv) * 1000.0) / (maxv - minv))];
						}
					}

					//draw

					if (LCDLay1FrameBuf == LCDLay1FrontBuf) {
						LCDLay1FrameBuf = LCDLay1BackBuf;
					} else {
						LCDLay1FrameBuf = LCDLay1FrontBuf;
					}

					for (int y = 0; y < 272; y++) {
						for (int x = 0; x < 480; x++) {
							int pixPos = (((272 - 1 - y) * 156) / 272) * 208 + ((x * 206) / 480);
							LCDLay1FrameBuf[(y * 480) + x] = FrameID3_cc[pixPos];
						}
					}

//					int minTemp = ((minv) - 5950) / 40;
//					int maxTemp = ((maxv - 100) - 5950) / 40;
//					char str[50];
//					sprintf(str, "max=%dc", maxTemp);
//					LCD_DrawString(&Font20, 0, 0, str);
//					sprintf(str, "min=%dc", minTemp);
//					LCD_DrawString(&Font20, 0, 20, str);
					HAL_LTDC_SetAddress(&hltdc, (uint32_t)LCDLay1FrameBuf, 0);
				}
					break;
				case 4: {
					memcpy(FrameID4, rawFrame, 64896);
					double gainFrameAvgVal = GetFrameMedian(FrameID4);
					for (int i = 0; i < 32448; i++) {
						if (FrameID4[i] > 2000 && FrameID4[i] < 8000) {
							GainCalibration[i] = gainFrameAvgVal
									/ (double) FrameID4[i];
						} else {
							GainCalibration[i] = 1.0;
							BadPixels[i] = 1;
						}
					}
				}
					break;
				default:
					break;
				}

				HAL_GPIO_WritePin(USB_PSO_GPIO_Port, USB_PSO_Pin, GPIO_PIN_RESET);

			}
		} else {
			//draw

			if (LCDLay1FrameBuf == LCDLay1FrontBuf) {
				LCDLay1FrameBuf = LCDLay1BackBuf;
			} else {
				LCDLay1FrameBuf = LCDLay1FrontBuf;
			}

			for (int y = 0; y < 272; y++) {
				for (int x = 0; x < 480; x++) {
					LCDLay1FrameBuf[(y * 480) + x] =  RainPallete565[100];
				}
			}

			char str[50];
			sprintf(str, "NO SIGNAL");
			LCD_DrawString(&Font20, 177, 126, str);

			HAL_LTDC_SetAddress(&hltdc, (uint32_t)LCDLay1FrameBuf, 0);

			HAL_Delay(100);
		}

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE()
	;

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType =
	/*RCC_OSCILLATORTYPE_HSI|*/RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLM = 10;
	RCC_OscInitStruct.PLL.PLLN = 216;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
	RCC_OscInitStruct.PLL.PLLQ = 9;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Activate the Over-Drive mode
	 */
	if (HAL_PWREx_EnableOverDrive() != HAL_OK) {
		Error_Handler();
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK) {
		Error_Handler();
	}

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_LTDC
			| RCC_PERIPHCLK_CLK48;
	PeriphClkInitStruct.PLLSAI.PLLSAIN = 144;
	PeriphClkInitStruct.PLLSAI.PLLSAIR = 4;
	PeriphClkInitStruct.PLLSAI.PLLSAIQ = 2;
	PeriphClkInitStruct.PLLSAI.PLLSAIP = RCC_PLLSAIP_DIV6;
	PeriphClkInitStruct.PLLSAIDivQ = 1;
	PeriphClkInitStruct.PLLSAIDivR = RCC_PLLSAIDIVR_8;
	PeriphClkInitStruct.Clk48ClockSelection = RCC_CLK48SOURCE_PLL;
	if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* DMA2D init function */
static void MX_DMA2D_Init(void)
{

  hdma2d.Instance = DMA2D;
  hdma2d.Init.Mode = DMA2D_M2M;
  hdma2d.Init.ColorMode = DMA2D_OUTPUT_RGB565;
  hdma2d.Init.OutputOffset = 0;
  hdma2d.LayerCfg[1].InputOffset = 0;
  hdma2d.LayerCfg[1].InputColorMode = DMA2D_INPUT_RGB565;
  hdma2d.LayerCfg[1].AlphaMode = DMA2D_NO_MODIF_ALPHA;
  hdma2d.LayerCfg[1].InputAlpha = 0;
  hdma2d.LayerCfg[1].AlphaInverted = DMA2D_REGULAR_ALPHA;
  hdma2d.LayerCfg[1].RedBlueSwap = DMA2D_RB_REGULAR;
  if (HAL_DMA2D_Init(&hdma2d) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_DMA2D_ConfigLayer(&hdma2d, 1) != HAL_OK)
  {
    Error_Handler();
  }

}

static void MX_LTDC_Init(void)
{

  LTDC_LayerCfgTypeDef pLayerCfg;


  hltdc.Instance = LTDC;
  hltdc.Init.HSPolarity = LTDC_HSPOLARITY_AL;
  hltdc.Init.VSPolarity = LTDC_VSPOLARITY_AL;
  hltdc.Init.DEPolarity = LTDC_DEPOLARITY_AL;
  hltdc.Init.PCPolarity = LTDC_PCPOLARITY_IPC;
  hltdc.Init.HorizontalSync = 40;
  hltdc.Init.VerticalSync = 9;
  hltdc.Init.AccumulatedHBP = 42;
  hltdc.Init.AccumulatedVBP = 11;
  hltdc.Init.AccumulatedActiveW = 522;
  hltdc.Init.AccumulatedActiveH = 283;
  hltdc.Init.TotalWidth = 524;
  hltdc.Init.TotalHeigh = 285;
  hltdc.Init.Backcolor.Blue = 0;
  hltdc.Init.Backcolor.Green = 0;
  hltdc.Init.Backcolor.Red = 0;
  if (HAL_LTDC_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }

  pLayerCfg.WindowX0 = 0;
  pLayerCfg.WindowX1 = 480;
  pLayerCfg.WindowY0 = 0;
  pLayerCfg.WindowY1 = 272;
  pLayerCfg.PixelFormat = LTDC_PIXEL_FORMAT_RGB565;
  pLayerCfg.Alpha = 255;
  pLayerCfg.Alpha0 = 0;
  pLayerCfg.BlendingFactor1 = LTDC_BLENDING_FACTOR1_CA;
  pLayerCfg.BlendingFactor2 = LTDC_BLENDING_FACTOR2_CA;
  pLayerCfg.FBStartAdress = (uint32_t)LCDLay1FrameBuf;
  pLayerCfg.ImageWidth = 480;
  pLayerCfg.ImageHeight = 272;
  pLayerCfg.Backcolor.Blue = 0;
  pLayerCfg.Backcolor.Green = 0;
  pLayerCfg.Backcolor.Red = 0;
  if (HAL_LTDC_ConfigLayer(&hltdc, &pLayerCfg, 0) != HAL_OK)
  {
    Error_Handler();
  }

}

/* SPI5 init function */
static void MX_SPI5_Init(void) {

	hspi5.Instance = SPI5;
	hspi5.Init.Mode = SPI_MODE_MASTER;
	hspi5.Init.Direction = SPI_DIRECTION_2LINES;
	hspi5.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi5.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi5.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi5.Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi5.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
	hspi5.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi5.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi5.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi5.Init.CRCPolynomial = 7;
	hspi5.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
	hspi5.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
	if (HAL_SPI_Init(&hspi5) != HAL_OK) {
		Error_Handler();
	}

}

/* TIM3 init function */
static void MX_TIM3_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;
	TIM_OC_InitTypeDef sConfigOC;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 10;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 256;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_PWM_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 0;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3)
			!= HAL_OK) {
		Error_Handler();
	}

	HAL_TIM_MspPostInit(&htim3);

}

/* TIM7 init function */
static void MX_TIM7_Init(void) {

	TIM_MasterConfigTypeDef sMasterConfig;

	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 1000;
	htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim7.Init.Period = 108;
	htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim7) != HAL_OK) {
		Error_Handler();
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}

}
/* FMC initialization function */
static void MX_FMC_Init(void) {
	FMC_SDRAM_TimingTypeDef SdramTiming;

	/** Perform the SDRAM1 memory initialization sequence
	 */
	hsdram1.Instance = FMC_SDRAM_DEVICE;
	/* hsdram1.Init */
	hsdram1.Init.SDBank = FMC_SDRAM_BANK2;
	hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_8;
	hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_12;
	hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
	hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
	hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_2;
	hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
	hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
	hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
	hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
	/* SdramTiming */
	SdramTiming.LoadToActiveDelay = 2;
	SdramTiming.ExitSelfRefreshDelay = 8;
	SdramTiming.SelfRefreshTime = 5;
	SdramTiming.RowCycleDelay = 7;
	SdramTiming.WriteRecoveryTime = 3;
	SdramTiming.RPDelay = 2;
	SdramTiming.RCDDelay = 2;

	if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK) {
		Error_Handler();
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOE_CLK_ENABLE()
	;
	__HAL_RCC_GPIOI_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOH_CLK_ENABLE()
	;
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;
	__HAL_RCC_GPIOB_CLK_ENABLE()
	;
	__HAL_RCC_GPIOG_CLK_ENABLE()
	;
	__HAL_RCC_GPIOD_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(USB_PSO_GPIO_Port, USB_PSO_Pin, GPIO_PIN_SET);

	/*Configure GPIO pin : LCD_PENIRQ_Pin */
	GPIO_InitStruct.Pin = LCD_PENIRQ_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_PENIRQ_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : LCD_BUSY_Pin */
	GPIO_InitStruct.Pin = LCD_BUSY_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(LCD_BUSY_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : USB_PSO_Pin */
	GPIO_InitStruct.Pin = USB_PSO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(USB_PSO_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct;

  /* Disables the MPU */
  HAL_MPU_Disable();
    /**Initializes and configures the Region and the memory to be protected
    */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0xD0000000;
  MPU_InitStruct.Size = MPU_REGION_SIZE_8MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /* Enables the MPU */
  HAL_MPU_Enable(MPU_HFNMI_PRIVDEF);

}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler */
	/* User can add his own implementation to report the HAL error return state */
	while (1) {
	}
	/* USER CODE END Error_Handler */
}

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
