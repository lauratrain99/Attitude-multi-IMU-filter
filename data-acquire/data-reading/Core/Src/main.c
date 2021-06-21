/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "spi.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>
#include "MPU9250_Config.h"

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
//uint8_t _buffer[21];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  // Record accelerometer, gyro and magnetometer data
  int16_t AccData[3] = {0, 0, 0} , GyroData[3] = {0, 0, 0}, MagData[3] = {0, 0, 0};

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
  MX_USART2_UART_Init();
  MX_SPI2_Init();

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  /* Buffers to store the print statements*/
  uint16_t ref = whoAmI();
  char kk[255];

  /* Lower CS for IMUs 2,3,4 */
  MPU9250_Deactivate(MPU9250_2_CS_PIN);
  MPU9250_Deactivate(MPU9250_3_CS_PIN);
  MPU9250_Deactivate(MPU9250_4_CS_PIN);

  /*Initialize the four IMUs */
  MPU9250_Init(MPU9250_1_CS_PIN);
  MPU9250_Init(MPU9250_2_CS_PIN);
  MPU9250_Init(MPU9250_3_CS_PIN);
  MPU9250_Init(MPU9250_4_CS_PIN);


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  /* Read IMU 1 */
	  MPU9250_GetData(AccData, GyroData, MagData, MPU9250_1_CS_PIN);

	  ref = sprintf(kk,"\t IMU 1 \r\n\n ax = %.3f  g's \r\n ay = %.3f  g's \r\n az = %.3f  g's \r\n wx = %.3f  deg/s \r\n wy = %.3f  deg/s \r\n wz = %.3f  deg/s \r\n mx = %.3f  µT \r\n my = %.3f  µT \r\n mz = %.3f  µT \r\n------------------ \r\n\n",
	  		(float)AccData[0]/2048.0, (float)AccData[1]/2048.0, (float)AccData[2]/2048.0,
	  		(float)GyroData[0]/16.4, (float)GyroData[1]/16.4, (float)GyroData[2]/16.4,
	  		(float)MagData[0]*0.6, (float)MagData[1]*0.6, (float)MagData[2]*0.6);


	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);
	  HAL_UART_Transmit(&huart2, kk, ref, 1000);

	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);
	  HAL_Delay(1000);


	  /* Read IMU 2 */
	  MPU9250_GetData(AccData, GyroData, MagData, MPU9250_2_CS_PIN);

	  ref = sprintf(kk,"\t IMU 2 \r\n\n ax = %.3f  g's \r\n ay = %.3f  g's \r\n az = %.3f  g's \r\n wx = %.3f  deg/s \r\n wy = %.3f  deg/s \r\n wz = %.3f  deg/s \r\n mx = %.3f  µT \r\n my = %.3f  µT \r\n mz = %.3f  µT \r\n------------------ \r\n\n",
		(float)AccData[0]/2048.0, (float)AccData[1]/2048.0, (float)AccData[2]/2048.0,
		(float)GyroData[0]/16.4, (float)GyroData[1]/16.4, (float)GyroData[2]/16.4,
		(float)MagData[0]*0.6, (float)MagData[1]*0.6, (float)MagData[2]*0.6);


	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);
	  HAL_UART_Transmit(&huart2, kk, ref, 1000);

	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);
	  HAL_Delay(1000);


	  /* Read IMU 3 */
	  MPU9250_GetData(AccData, GyroData, MagData, MPU9250_3_CS_PIN);

	  ref = sprintf(kk,"\t IMU 3 \r\n\n ax = %.3f  g's \r\n ay = %.3f  g's \r\n az = %.3f  g's \r\n wx = %.3f  deg/s \r\n wy = %.3f  deg/s \r\n wz = %.3f  deg/s \r\n mx = %.3f  µT \r\n my = %.3f  µT \r\n mz = %.3f  µT \r\n------------------ \r\n\n",
			(float)AccData[0]/2048.0, (float)AccData[1]/2048.0, (float)AccData[2]/2048.0,
			(float)GyroData[0]/16.4, (float)GyroData[1]/16.4, (float)GyroData[2]/16.4,
			(float)MagData[0]*0.6, (float)MagData[1]*0.6, (float)MagData[2]*0.6);


	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);
	  HAL_UART_Transmit(&huart2, kk, ref, 1000);

	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);
	  HAL_Delay(1000);


	  /* Read IMU 4 */
	  MPU9250_GetData(AccData, GyroData, MagData, MPU9250_4_CS_PIN);

	  ref = sprintf(kk,"\t IMU 4 \r\n\n ax = %.3f  g's \r\n ay = %.3f  g's \r\n az = %.3f  g's \r\n wx = %.3f  deg/s \r\n wy = %.3f  deg/s \r\n wz = %.3f  deg/s \r\n mx = %.3f  µT \r\n my = %.3f  µT \r\n mz = %.3f  µT \r\n------------------ \r\n\n",
			(float)AccData[0]/2048.0, (float)AccData[1]/2048.0, (float)AccData[2]/2048.0,
			(float)GyroData[0]/16.4, (float)GyroData[1]/16.4, (float)GyroData[2]/16.4,
			(float)MagData[0]*0.6, (float)MagData[1]*0.6, (float)MagData[2]*0.6);


	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_SET);
	  HAL_UART_Transmit(&huart2, kk, ref, 1000);

	  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5, GPIO_PIN_RESET);
	  HAL_Delay(1000);


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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
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
