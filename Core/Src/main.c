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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ssd1306.h"
#include "ms8607.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MS8607_ADDRESS 0x76

// Command Definitions
#define CMD_READ_PROM 0xA0
#define CMD_READ_TEMP 0x58
#define CMD_READ_PRESS 0x40
#define CMD_RESET 0x1E

#define INA260_I2C_ADDRESS  0x40 << 1
#define REG_CURRENT         0x01
#define REG_BUS_VOLTAGE     0x02
#define REG_POWER           0x03
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
I2C_HandleTypeDef hi2c3;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void SystemPower_Config(void);
static void MX_GPIO_Init(void);
static void MX_ICACHE_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(int baud);
static void MX_UART5_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C3_Init(void);

/* USER CODE BEGIN PFP */
int presence = 0, isRxed = 0;
  uint8_t RxData[8], Temp_LSB = 0, Temp_MSB = 0;
  int16_t Temp;
  float temp_DS18B20;
  void INA260_Init();
  float INA260_ReadCurrent();
  float INA260_ReadVoltage();
  float INA260_ReadPower();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int DS18B20_start (void)
{
		uint8_t data = 0xF0;
		MX_UART4_Init(9600);
		HAL_UART_Transmit(&huart4, &data, 1, 100);
		if (HAL_UART_Receive(&huart4, &data, 1, 1000) != HAL_OK) return -1; // failed.. check connection
		MX_UART4_Init(115200);
		if (data == 0xf0) return -2; //error no device detected
		return 1;
}

void DS18B20_Write (uint8_t data)
{
	uint8_t buffer[8];
	for (int i = 0; i < 8; i++) {
		if ((data & (1<<i)) != 0){  // if the bit is high
			buffer[i] = 0xFF; // send a 1
		} else {
			buffer[i] = 0x00; // send a 0
		}
	}

	HAL_UART_Transmit(&huart4, buffer, 8, 1000);
}

uint8_t DS18B20_Read(void)
{
    uint8_t value = 0;
    for (int i = 0; i < 8; i++)
    {
        uint8_t tx_byte = 0xFF;  // Transmit request for a read time slot
        uint8_t rx_byte = 0;

        // Transmit and receive each bit
        if (HAL_UART_Transmit(&huart4, &tx_byte, 1, 1000) != HAL_OK)
        {
            // Handle transmission error
        }

        if (HAL_UART_Receive(&huart4, &rx_byte, 1, 1000) != HAL_OK)
        {
            // Handle reception error
        }

        // Store the received bit in RxData for debugging
        RxData[i] = rx_byte;

        // Interpret received bit as 1 if rx_byte == 0xFF, otherwise 0
        if (rx_byte == 0xFF)
        {
            value |= (1 << i); // Set the bit in value
        }
    }
    return value;
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	isRxed = 1;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	uint16_t prom[6];

	  char snum[15] = "Hello world";
	  double temperature;
	  float pressure;
	  float humidity;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* Configure the System Power */
  SystemPower_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ICACHE_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_UART4_Init(9600);
  MX_UART5_Init();
  MX_USART3_UART_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */
  SSD1306_Init();
  INA260_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
    {
	  MS8607_ReadPROM(&hi2c2, prom);
	  temperature = MS8607_ReadTemperature(&hi2c2, prom);

	    	  // Convert temperature to string for display
	    	  snprintf(snum, sizeof(snum), "%.2lf C", temperature);

	    	  // Update the display
	    	  SSD1306_GotoXY(0, 0);
	    	  SSD1306_Puts(snum, &Font_7x10, SSD1306_COLOR_WHITE);
	    	  SSD1306_UpdateScreen();
	    	  strcat(snum, "\n\r");
	    	  HAL_UART_Transmit(&huart5, (uint8_t*)snum, strlen(snum), 100);
	    	  pressure = MS8607_ReadPressure(&hi2c2, prom, temperature);
	    	  // Convert temperature to string for display
	    	  snprintf(snum, sizeof(snum), "%.2f HPa", pressure);

	    	  // Update the display
	    	  SSD1306_GotoXY(0, 20);
	    	  SSD1306_Puts(snum, &Font_7x10, SSD1306_COLOR_WHITE);
	    	  SSD1306_UpdateScreen();
	    	  strcat(snum, "\n\r");
	    	  HAL_UART_Transmit(&huart5, (uint8_t*)snum, strlen(snum), 100);
	    	  humidity = MS8607_ReadHumidity(&hi2c2);
	    	  // Convert temperature to string for display
	    	  snprintf(snum, sizeof(snum), "%.2f%%", humidity);
	    	  // Update the display
	    	  SSD1306_GotoXY(0, 40);
	    	  SSD1306_Puts(snum, &Font_7x10, SSD1306_COLOR_WHITE);
	    	  SSD1306_UpdateScreen();
	    	  strcat(snum, "\n\r");
	    	  HAL_UART_Transmit(&huart5, (uint8_t*)snum, strlen(snum), 100);
	    	  presence = DS18B20_start();
	    	  	  DS18B20_Write(0xCC); // skip ROM
	    	  	  DS18B20_Write(0x44); // convert temperature

	    	  	  presence = DS18B20_start();
	    	  	  DS18B20_Write(0xCC); // skip ROM
	    	  	  DS18B20_Write(0xBE); // read scratchpad

	    	  	  Temp_LSB = DS18B20_Read();
	    	  	  Temp_MSB = DS18B20_Read();
	    	  	  Temp = (Temp_MSB<<8) | Temp_LSB;
	    	  	  temp_DS18B20 = (float)Temp/16.0; // resolution is 0.0625
	    	  	snprintf(snum, sizeof(snum), "%.2f C Ice", temp_DS18B20);

	    	  		    	  // Update the display
	    	  		    	  SSD1306_GotoXY(50, 40);
	    	  		    	  SSD1306_Puts(snum, &Font_7x10, SSD1306_COLOR_WHITE);
	    	  		    	  SSD1306_UpdateScreen();
	    	  		    	  strcat(snum, "\n\r");
	    	  		    	  HAL_UART_Transmit(&huart5, (uint8_t*)snum, strlen(snum), 100);
	    	  		    	HAL_Delay(1000);
	    	  		    	SSD1306_Fill(SSD1306_COLOR_BLACK);
	    	  		    	float current = INA260_ReadCurrent();
	    	  		    	snprintf(snum, sizeof(snum), "%.2f A", current);
	    	  		    	SSD1306_GotoXY(0, 0);
	    	  		    		    	  		    	  SSD1306_Puts(snum, &Font_7x10, SSD1306_COLOR_WHITE);
	    	  		    		    	  		    	  SSD1306_UpdateScreen();
	    	  		    	        float voltage = INA260_ReadVoltage();
	    	  		    	      snprintf(snum, sizeof(snum), "%.2f V", voltage);
	    	  		    	      SSD1306_GotoXY(0, 20);
	    	  		    	      	    	  		    	  SSD1306_Puts(snum, &Font_7x10, SSD1306_COLOR_WHITE);
	    	  		    	      	    	  		    	  SSD1306_UpdateScreen();
	    	  		    	        float power = INA260_ReadPower();
	    	  		    	      snprintf(snum, sizeof(snum), "%.2f W", power);
	    	  		    	      SSD1306_GotoXY(0, 40);
	    	  		    	      	    	  		    	  SSD1306_Puts(snum, &Font_7x10, SSD1306_COLOR_WHITE);
	    	  		    	      	    	  		    	  SSD1306_UpdateScreen();
	    	  HAL_Delay(1000); // Update every second
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

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_0;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLMBOOST = RCC_PLLMBOOST_DIV4;
  RCC_OscInitStruct.PLL.PLLM = 3;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 1;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLLVCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Power Configuration
  * @retval None
  */
static void SystemPower_Config(void)
{

  /*
   * Switch to SMPS regulator instead of LDO
   */
  if (HAL_PWREx_ConfigSupply(PWR_SMPS_SUPPLY) != HAL_OK)
  {
    Error_Handler();
  }
/* USER CODE BEGIN PWR */
/* USER CODE END PWR */
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00F07BFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00F07BFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief I2C3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C3_Init(void)
{

  /* USER CODE BEGIN I2C3_Init 0 */

  /* USER CODE END I2C3_Init 0 */

  /* USER CODE BEGIN I2C3_Init 1 */

  /* USER CODE END I2C3_Init 1 */
  hi2c3.Instance = I2C3;
  hi2c3.Init.Timing = 0x30909DEC;
  hi2c3.Init.OwnAddress1 = 0;
  hi2c3.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c3.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c3.Init.OwnAddress2 = 0;
  hi2c3.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c3.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c3.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c3, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c3, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C3_Init 2 */

  /* USER CODE END I2C3_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */

  /** Enable instruction cache in 1-way (direct mapped cache)
  */
  if (HAL_ICACHE_ConfigAssociativityMode(ICACHE_1WAY) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_ICACHE_Enable() != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(int baud)
{

  /* USER CODE BEGIN UART4_Init 0 */
  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */
  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = baud;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */
  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart5, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart5, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

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
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_HalfDuplex_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void INA260_Init() {
    // INA260 uses default configuration; no initialization required.
	 uint8_t config_data[] = {0x00, 0x00};  // Example configuration
	    HAL_I2C_Master_Transmit(&hi2c3, INA260_I2C_ADDRESS << 1, config_data, sizeof(config_data), HAL_MAX_DELAY);
}

// Read raw data from a register
uint16_t INA260_ReadRegister(uint8_t reg) {
    uint8_t data[2] = {0};
    HAL_I2C_Master_Transmit(&hi2c3, INA260_I2C_ADDRESS, &reg, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(&hi2c3, INA260_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);

    return (data[0] << 8) | data[1];
}

// Convert current register value to amperes
float INA260_ReadCurrent() {
    int16_t raw_current = (int16_t)INA260_ReadRegister(REG_CURRENT);
    return raw_current * 0.00125;  // Conversion factor: 1.25 mA/bit
}

// Convert voltage register value to volts
float INA260_ReadVoltage() {
    uint16_t raw_voltage = INA260_ReadRegister(REG_BUS_VOLTAGE);
    return raw_voltage * 0.00125;  // Conversion factor: 1.25 mV/bit
}

// Convert power register value to watts
float INA260_ReadPower() {
    uint16_t raw_power = INA260_ReadRegister(REG_POWER);
    return raw_power * 0.01;  // Conversion factor: 10 mW/bit
}
/* USER CODE END 4 */

/**
  * @brief BSP Push Button callback
  * @param Button Specifies the pressed button
  * @retval None
  */


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
