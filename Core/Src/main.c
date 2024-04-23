/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "spi.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "adis16477.h"

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
void PeriphCommonClock_Config(void);
/* USER CODE BEGIN PFP */

int8_t ADIS_read 		(uint16_t reg_addr, uint16_t *read_data, uint16_t len);
int8_t ADIS_burst_read	(uint16_t reg_addr, uint16_t *read_data, uint16_t len);
int8_t ADIS_write 		(uint16_t reg_addr, const uint16_t *read_data, uint16_t len);
void   ADIS_Delay		(uint32_t period);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float X_GYRO;
float Y_GYRO;
float Z_GYRO;
float X_ACCEL;
float Y_ACCEL;
float Z_ACCEL;
float ADIS_TEMP;
float start_time, end_time, execution_time, freq;
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  // ADIS-16477
  struct ADIS ADIS_dev;

  /* Bus configuration : SPI */

  ADIS_dev.read = 		ADIS_read;
  ADIS_dev.burst_read = ADIS_burst_read;
  ADIS_dev.write = 		ADIS_write;
  ADIS_dev.delay = 		ADIS_Delay;

  // Hard reset ADIS
  // Change the values in header file according to your setup
  HAL_GPIO_WritePin(HARD_RST, HARD_RST_PIN, 0);
  HAL_Delay(1);
  HAL_GPIO_WritePin(HARD_RST, HARD_RST_PIN, 1);

  ADIS_16477_Init(&ADIS_dev);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  start_time = HAL_GetTick();

	  Burst_Read_16(&ADIS_dev);
	  X_GYRO = Scale_gyro (ADIS_dev.gyro_x);
	  Y_GYRO = Scale_gyro (ADIS_dev.gyro_y);
	  Z_GYRO = Scale_gyro (ADIS_dev.gyro_z);
	  X_ACCEL = Scale_accel (ADIS_dev.accel_x);
      Y_ACCEL = Scale_accel (ADIS_dev.accel_y);
      Z_ACCEL = Scale_accel (ADIS_dev.accel_z);
      ADIS_TEMP = Scale_temp (ADIS_dev.temp);

	  end_time = HAL_GetTick();
	  execution_time = end_time - start_time;
	  execution_time = execution_time / 1000;
	  freq = 1 / execution_time;

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

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV1;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInitStruct.CkperClockSelection = RCC_CLKPSOURCE_HSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int8_t ADIS_read (uint16_t reg_addr, uint16_t *read_data, uint16_t len){

  uint16_t txData = ((reg_addr & 0x7F) << 8);

  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_SET);
  HAL_Delay(10);
  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_RESET);

  if (HAL_SPI_Transmit(ADIS_SPI, (uint8_t *)&txData, 1, 1000) != HAL_OK) {
    return -9;
  }
  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_SET);
  HAL_Delay (1);
  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_RESET);


  if (HAL_SPI_Receive(ADIS_SPI, (uint8_t *)read_data, len, 1000) != HAL_OK) {
      return -9;
  }

  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_SET);
  HAL_Delay (1);
  return 0;
  }

int8_t ADIS_burst_read	(uint16_t reg_addr, uint16_t *read_data, uint16_t len){

  uint16_t txData = ((reg_addr & 0x7F) << 8);

    HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_RESET);
    if (HAL_SPI_Transmit(ADIS_SPI, (uint8_t *)&txData, 1, 1000) != HAL_OK) {
        return -9;
      }
    if (HAL_SPI_Receive(ADIS_SPI, (uint8_t *) read_data, len, 1000) != HAL_OK) {
          return -9;
      }

      HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_SET);
      HAL_Delay(1);
      return 0;

}

int8_t ADIS_write (uint16_t reg_addr, const uint16_t *read_data, uint16_t len){

  uint16_t txData = (((reg_addr & 0x7F) | 0x80) << 8);
  uint16_t CMD = *read_data;
  uint16_t lowbyte = (txData | (CMD & 0xFF));
  uint16_t highbyte = ((txData | 0x100) | ((CMD >> 8) & 0xFF));
  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_SET);
  HAL_Delay(1);
  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_RESET);

   if (HAL_SPI_Transmit(ADIS_SPI, (uint8_t *) &lowbyte, 1, 1000) != HAL_OK) {
     return -9;
   }
  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_SET);
  HAL_Delay (1);
  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_RESET);

   if (HAL_SPI_Transmit(ADIS_SPI, (uint8_t *) &highbyte, 1, 1000) != HAL_OK) {
      return -9;
     }
  HAL_GPIO_WritePin(ADIS_CS, ADIS_CS_PIN, GPIO_PIN_SET);
  HAL_Delay (1);
return 0;
}
void ADIS_Delay(uint32_t period){
HAL_Delay(period);
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
