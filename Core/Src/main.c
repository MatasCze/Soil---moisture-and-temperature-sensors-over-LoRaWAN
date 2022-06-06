/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

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
 ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
long map(long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int __io_putchar(int ch)
{
 HAL_UART_Transmit(&huart1, (uint8_t*)&ch, 1, HAL_MAX_DELAY);
 return 1;
}

static void set_baudrate(uint32_t baudrate)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = baudrate;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;

  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}


HAL_StatusTypeDef wire_reset(void)
{
  uint8_t data_out = 0xF0;
  uint8_t data_in = 0;

  set_baudrate(9600);
  HAL_UART_Transmit(&huart3, &data_out, 1, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart3, &data_in, 1, HAL_MAX_DELAY);
  set_baudrate(115200);

  if (data_in != 0xF0)
    return HAL_OK;
  else
    return HAL_ERROR;
}

static int read_bit(void)
{
  uint8_t data_out = 0xFF;
  uint8_t data_in = 0;
  HAL_UART_Transmit(&huart3, &data_out, 1, HAL_MAX_DELAY);
  HAL_UART_Receive(&huart3, &data_in, 1, HAL_MAX_DELAY);

  return data_in & 0x01;
}

uint8_t wire_read(void)
{
  uint8_t value = 0;
  int i;
  for (i = 0; i < 8; i++) {
    value >>= 1;
    if (read_bit())
      value |= 0x80;
  }
  return value;
}

static void write_bit(int value)
{
  if (value) {
      uint8_t data_out = 0xff;
    HAL_UART_Transmit(&huart3, &data_out, 1, HAL_MAX_DELAY);
  } else {
      uint8_t data_out = 0x0;
    HAL_UART_Transmit(&huart3, &data_out, 1, HAL_MAX_DELAY);
  }
}

void wire_write(uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    write_bit(byte & 0x01);
    byte >>= 1;
  }
}

static uint8_t byte_crc(uint8_t crc, uint8_t byte)
{
  int i;
  for (i = 0; i < 8; i++) {
    uint8_t b = crc ^ byte;
    crc >>= 1;
    if (b & 0x01)
      crc ^= 0x8c;
    byte >>= 1;
  }
  return crc;
}

uint8_t wire_crc(const uint8_t* data, int len)
{
  int i;
    uint8_t crc = 0;

    for (i = 0; i < len; i++)
      crc = byte_crc(crc, data[i]);

    return crc;
}

#define DS18B20_ROM_CODE_SIZE		8
#define DS18B20_SCRATCHPAD_SIZE    9

#define DS18B20_READ_ROM           0x33
#define DS18B20_MATCH_ROM          0x55
#define DS18B20_SKIP_ROM           0xCC

#define DS18B20_CONVERT_T          0x44
#define DS18B20_READ_SCRATCHPAD    0xBE

HAL_StatusTypeDef ds18b20_init(void)
{
  return wire_init();
}

HAL_StatusTypeDef ds18b20_read_address(uint8_t* rom_code)
{
  int i;
  uint8_t crc;

  if (wire_reset() != HAL_OK)
    return HAL_ERROR;

  wire_write(DS18B20_READ_ROM);

  for (i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
    rom_code[i] = wire_read();

  crc = wire_crc(rom_code, DS18B20_ROM_CODE_SIZE - 1);
  if (rom_code[DS18B20_ROM_CODE_SIZE - 1] == crc)
    return HAL_OK;
  else
    return HAL_ERROR;
}

static HAL_StatusTypeDef send_cmd(const uint8_t* rom_code, uint8_t cmd)
{
  int i;

  if (wire_reset() != HAL_OK)
    return HAL_ERROR;

  if (!rom_code) {
    wire_write(DS18B20_SKIP_ROM);
  } else {
    wire_write(DS18B20_MATCH_ROM);
    for (i = 0; i < DS18B20_ROM_CODE_SIZE; i++)
      wire_write(rom_code[i]);
  }
  wire_write(cmd);
  return HAL_OK;
}

HAL_StatusTypeDef ds18b20_start_measure(const uint8_t* rom_code)
{
  return send_cmd(rom_code, DS18B20_CONVERT_T);
}

static HAL_StatusTypeDef ds18b20_read_scratchpad(const uint8_t* rom_code, uint8_t* scratchpad)
{
  int i;
  uint8_t crc;

  if (send_cmd(rom_code, DS18B20_READ_SCRATCHPAD) != HAL_OK)
    return HAL_ERROR;

  for (i = 0; i < DS18B20_SCRATCHPAD_SIZE; i++)
    scratchpad[i] = wire_read();

  crc = wire_crc(scratchpad, DS18B20_SCRATCHPAD_SIZE - 1);
  if (scratchpad[DS18B20_SCRATCHPAD_SIZE - 1] == crc)
    return HAL_OK;
  else
    return HAL_ERROR;
}

float ds18b20_get_temp(const uint8_t* rom_code)
{
  uint8_t scratchpad[DS18B20_SCRATCHPAD_SIZE];
  int16_t temp;

  if (ds18b20_read_scratchpad(rom_code, scratchpad) != HAL_OK)
    return 85.0f;

  memcpy(&temp, &scratchpad[0], sizeof(temp));

  return temp / 16.0f;
}




/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t inputValueMoisture;
	uint16_t inputValueTemp;
	uint16_t percent;
	const uint16_t dry = 2490;
	const uint16_t wet = 1531;

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
  MX_ADC1_Init();
  MX_USART3_UART_Init();
  MX_RTC_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  int connection_iter = 0;
  HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
  if(HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 3600, RTC_WAKEUPCLOCK_CK_SPRE_16BITS) != HAL_OK)
   {
   Error_Handler();
   }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  ds18b20_start_measure(NULL);
	  HAL_Delay(750);

	  HAL_ADC_Start(&hadc1);

	  HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	  inputValueMoisture = HAL_ADC_GetValue(&hadc1);
	  percent = map(inputValueMoisture, wet, dry, 100, 0);

	  float temperature = ds18b20_get_temp(NULL);
	  if(percent > 100){
		  percent = 0;
	  }
	  //printf("Moisture of soil is %d %%, temperature is %.1f*C \n", percent, temp);


	  if(connection_iter < 1) {
	    		/* Connection */
	    		do {
	    			/* TX
	    				AT+ID=DevEui
	    			*/

	    			printf("AT+ID=DevEui\r\n");
	    			HAL_Delay(100);
	    			/* RX
	    				+ID: DevEui, 2C:F7:F1:20:24:90:07:11
	    			*/
	    			/* TX
	    				AT+ID=AppEui
	    			*/
	    			printf("AT+ID=AppEui\r\n");
	    			HAL_Delay(100);
	    			/* RX
	    				+ID: AppEui, 00:00:00:00:00:00:00:00
	    			*/
	    			/* TX
	    				AT+DR=EU868
	    			*/
	    			printf("AT+DR=EU868\r\n"); // frequency 868
	    			HAL_Delay(100);
	    			/* RX
	    				+DR: EU868
	    			*/
	    			/* Tx
	    				AT+CH=NUM,0-2
	    			*/
	    			printf("AT+CH=NUM,0-2\r\n"); // channel
	    			HAL_Delay(100);
	    			/* RX
	    				+CH: NUM, 0-2
	    			*/
	    			/* Tx
	    				AT+MODE=LWOTAA
	    			*/
	    			printf("AT+MODE=LWOTAA\r\n");
	    			HAL_Delay(100);
	    			/* RX
	    				+MODE: LWOTAA
	    			*/
	    			/* Tx
	    				AT+KEY=APPKEY
	    			*/
	    			printf("AT+KEY=APPKEY,\"402A1CF34012B797D09BF245931733D2\"\r\n");
	    			HAL_Delay(100);
	    			/* RX
	    				+KEY: APPKEY B71DED6CDA06912D9800C1031FBC5828
	    			*/
	    			/* Tx
	    				AT+ID
	    			*/
	    			printf("AT+ID\r\n");
	    			HAL_Delay(100);
	    			/* RX
	    				+ID: DevAddr, 24:90:07:11
	    				+ID: DevEui, 2C:F7:F1:20:24:90:07:11
	    				+ID: AppEui, 00:00:00:00:00:00:00:00
	    			*/
	    			/* Tx
	    				AT+JOIN
	    			*/
	    			printf("AT+JOIN\r\n");
	    			HAL_Delay(10000);
	    			/* RX
	    				+JOIN: Start
	    				+JOIN: NORMAL
	    				+JOIN: Network joined
	    				+JOIN: NetID 000013 DevAddr 26:01:5F:66
	    				+JOIN: Done
	    			*/

	    			connection_iter++;

	    		}
	    		while (connection_iter < 1);
	    	}

	  int soilhumi = percent;
	  int soiltemp = temperature * 10;

	    	//printf("AT+MSG=%f_%f\r\n", temp, humi);
	  printf("AT+MSG=%03d_%03d\r\n", soiltemp, soilhumi);

	  HAL_Delay(10);



	  HAL_PWR_EnterSTANDBYMode();



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
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSE
                              |RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV8;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enable MSI Auto calibration
  */
  HAL_RCCEx_EnableMSIPLLMode();
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

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutRemap = RTC_OUTPUT_REMAP_NONE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_RXOVERRUNDISABLE_INIT;
  huart3.AdvancedInit.OverrunDisable = UART_ADVFEATURE_OVERRUN_DISABLE;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
