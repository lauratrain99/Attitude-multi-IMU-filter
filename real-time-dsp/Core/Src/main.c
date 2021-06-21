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
#define ARM_MATH_CM4
#include "arm_math.h"
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
/* Private variables ---------------------------------------------------------*/

	float sum = 0;
	int m = 24;
	int n = 24;
	int c, d, k;
	int i,j;

	float m1[24][24] ={
	     1.0,     32.0,      4.0,     128.0,    3.0,      32.0,      4.0,     128.0,    52.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     32.0,     64.0,    2048.0,    5.0,      32.0,     64.0,     248.0,     3.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     16.0,      4.0,      64.0,    7.0,      16.0,      4.0,      64.0,    32.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     16.0,     64.0,    1024.0,    11.0,     16.0,     64.0,     124.0,    32.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     32.0,      4.0,     128.0,    13.0,    352.0,      4.0,     128.0,   322.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     32.0,     64.0,    2048.0,    15.0,    322.0,     64.0,     208.0,     2.0,    35.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     16.0,      4.0,      64.0,    17.0,    106.0,      4.0,      64.0,   352.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     16.0,     64.0,    1024.0,    21.0,    126.0,     64.0,     104.0,    32.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     8.0,     126.0,    64.0,    1024.0,    10.0,     12.0,     64.0,     102.0,    37.0,    38.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     32.0,      4.0,     128.0,     3.0,     32.0,      4.0,     128.0,    52.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     32.0,     64.0,    2048.0,     5.0,     32.0,     64.0,     248.0,     3.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     16.0,      4.0,      64.0,     7.0,     16.0,      4.0,      64.0,    32.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     16.0,     64.0,    1024.0,    11.0,     16.0,     64.0,     124.0,    32.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     32.0,      4.0,     128.0,    13.0,    352.0,      4.0,     128.0,   322.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     32.0,     64.0,    2048.0,    15.0,    322.0,     64.0,     208.0,     2.0,    35.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     16.0,      4.0,      64.0,    17.0,    106.0,      4.0,      64.0,   352.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     1.0,     16.0,     64.0,    1024.0,    21.0,    126.0,     64.0,     104.0,    32.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	     8.0,     126.0,    64.0,    14.0,      10.0,     12.0,     64.0,     102.0,    37.0,    38.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	   	 1.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     69.0,     124.0,    35.0,    22.0,     5.0,     17.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    4.0,    4.0,   7.0,
	   	 5.0,     16.0,     64.0,    1024.0,    12.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     68.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
	   	 5.0,     16.0,     64.0,    1024.0,    11.0,     13.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,     104.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   8.0,

	   };

	float m2[24][24]  = {
		     1.0,     32.0,      4.0,     128.0,    3.0,      32.0,      4.0,     128.0,    52.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     32.0,     64.0,    2048.0,    5.0,      32.0,     64.0,     248.0,     3.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     16.0,      4.0,      64.0,    7.0,      16.0,      4.0,      64.0,    32.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     16.0,     64.0,    1024.0,    11.0,     16.0,     64.0,     124.0,    32.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     32.0,      4.0,     128.0,    13.0,    352.0,      4.0,     128.0,   322.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     32.0,     64.0,    2048.0,    15.0,    322.0,     64.0,     208.0,     2.0,    35.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     16.0,      4.0,      64.0,    17.0,    106.0,      4.0,      64.0,   352.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     16.0,     64.0,    1024.0,    21.0,    126.0,     64.0,     104.0,    32.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     8.0,     126.0,    64.0,    1024.0,    10.0,     12.0,     64.0,     102.0,    37.0,    38.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     32.0,      4.0,     128.0,     3.0,     32.0,      4.0,     128.0,    52.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     32.0,     64.0,    2048.0,     5.0,     32.0,     64.0,     248.0,     3.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     16.0,      4.0,      64.0,     7.0,     16.0,      4.0,      64.0,    32.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     16.0,     64.0,    1024.0,    11.0,     16.0,     64.0,     124.0,    32.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     32.0,      4.0,     128.0,    13.0,    352.0,      4.0,     128.0,   322.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     32.0,     64.0,    2048.0,    15.0,    322.0,     64.0,     208.0,     2.0,    35.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     16.0,      4.0,      64.0,    17.0,    106.0,      4.0,      64.0,   352.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     1.0,     16.0,     64.0,    1024.0,    21.0,    126.0,     64.0,     104.0,    32.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		     8.0,     126.0,    64.0,    1024.0,    10.0,     12.0,     64.0,     102.0,    37.0,    38.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
		   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,

		   };

	float multiply[24][24];
	float ratio, a;
	float trans[24][24];







   /* --------------------------------------------------------------------------------
   * Test input data(Cycles) taken from FIR Q15 module for differant cases of blockSize
   * and tapSize
   * --------------------------------------------------------------------------------- */
   float32_t B_f32[24] =
   {
     782.0, 7577.0, 470.0, 4505.0, 470.0, 470.0, 470.0, 470.0, 782.0, 89, 782.0, 7577.0, 470.0, 4505.0, 470.0, 470.0, 470.0, 470.0, 782.0, 8, 9, 9, 9, 9};
   /* --------------------------------------------------------------------------------
   * Formula to fit is  C1 + C2 * numTaps + C3 * blockSize + C4 * numTaps * blockSize
   * -------------------------------------------------------------------------------- */
   float32_t A_f32[576] =
   {
     /* Const,   numTaps,   blockSize,   numTaps*blockSize */
     1.0,     32.0,      4.0,     128.0,    3.0,      32.0,      4.0,     128.0,    52.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     32.0,     64.0,    2048.0,    5.0,      32.0,     64.0,     248.0,     3.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     16.0,      4.0,      64.0,    7.0,      16.0,      4.0,      64.0,    32.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     16.0,     64.0,    1024.0,    11.0,     16.0,     64.0,     124.0,    32.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     32.0,      4.0,     128.0,    13.0,    352.0,      4.0,     128.0,   322.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     32.0,     64.0,    2048.0,    15.0,    322.0,     64.0,     208.0,     2.0,    35.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     16.0,      4.0,      64.0,    17.0,    106.0,      4.0,      64.0,   352.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     16.0,     64.0,    1024.0,    21.0,    126.0,     64.0,     104.0,    32.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     8.0,     126.0,    64.0,    1024.0,    10.0,     12.0,     64.0,     102.0,    37.0,    38.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     32.0,      4.0,     128.0,     3.0,     32.0,      4.0,     128.0,    52.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     32.0,     64.0,    2048.0,     5.0,     32.0,     64.0,     248.0,     3.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     16.0,      4.0,      64.0,     7.0,     16.0,      4.0,      64.0,    32.0,    32.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     16.0,     64.0,    1024.0,    11.0,     16.0,     64.0,     124.0,    32.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     32.0,      4.0,     128.0,    13.0,    352.0,      4.0,     128.0,   322.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     32.0,     64.0,    2048.0,    15.0,    322.0,     64.0,     208.0,     2.0,    35.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     16.0,      4.0,      64.0,    17.0,    106.0,      4.0,      64.0,   352.0,    52.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     1.0,     16.0,     64.0,    1024.0,    21.0,    126.0,     64.0,     104.0,    32.0,   352.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
     8.0,     126.0,    64.0,    1024.0,    10.0,     12.0,     64.0,     102.0,    37.0,    38.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,
   	 5.0,     16.0,     64.0,    1024.0,    11.0,     11.0,     67.0,     124.0,    35.0,    22.0,     5.0,     16.0,     64.0,    1024.0,     11.0,     11.0,     67.0,    124.0,     35.0,     2.0,     2.0,    3.0,    4.0,   7.0,

   };
   /* ----------------------------------------------------------------------
   * Save matrix for home-made approach
   * ------------------------------------------------------------------- */


   /* ----------------------------------------------------------------------
   * Temporary buffers  for storing intermediate values
   * ------------------------------------------------------------------- */
   /* Transpose of A Buffer */
   float32_t AT_f32[576];
   /* Inverse of A Buffer */
   float32_t AI_f32[576];
   /* (Transpose of A * A) Buffer */
   float32_t ATMA_f32[576];
   /* Inverse(Transpose of A * A)  Buffer */
   float32_t ATMAI_f32[576];
   /* Test Output Buffer */
   float32_t X_f32[24];
   /* ----------------------------------------------------------------------
   * Reference ouput buffer C1, C2, C3 and C4 taken from MATLAB
   * ------------------------------------------------------------------- */
   float32_t xRef_f32[24] = {73.0, 8.0, 21.25, 2.875, 75.0, 8.0, 21.25, 2.875, 73.0, 8.0, 73.0, 8.0, 21.25, 2.875, 75.0, 8.0, 21.25, 2.875, 7.0, 3.0, 8.0, 5.0, 9, 22.0};
   float32_t snr;

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
  arm_matrix_instance_f32 A;          /* Matrix A Instance */
  arm_matrix_instance_f32 AT;         /* Matrix AT(A transpose) instance */
  arm_matrix_instance_f32 AI;         /* Matrix AI(A inverse) instance */
  arm_matrix_instance_f32 ATMA;       /* Matrix ATMA( AT multiply with A) instance */
  arm_matrix_instance_f32 ATMAI;      /* Matrix ATMAI(Inverse of ATMA) instance */
  arm_matrix_instance_f32 B;          /* Matrix B instance */
  arm_matrix_instance_f32 X;          /* Matrix X(Unknown Matrix) instance */
  //arm_matrix_instance_f32 K;          /* Matrix K(B multiplied by AT times the inverse of (B + A*B+AT) instance */
  uint32_t srcRows, srcColumns;       /* Temporary variables */
  arm_status status;

  /* Initialise Matrix Instance AT with numRows, numCols and data array(AT_f32) */
  srcRows = 24;
  srcColumns = 24;
  arm_mat_init_f32(&A, srcRows, srcColumns, A_f32);
  arm_mat_init_f32(&AT, srcRows, srcColumns, AT_f32);
  arm_mat_init_f32(&AI, srcRows, srcColumns, AI_f32);

  /* Initialise ATMA Matrix Instance with numRows, numCols and data array(ATMA_f32) */
  srcRows = 24;
  srcColumns = 24;
  arm_mat_init_f32(&ATMA, srcRows, srcColumns, ATMA_f32);
  /* Initialise ATMAI Matrix Instance with numRows, numCols and data array(ATMAI_f32) */
  srcRows = 24;
  srcColumns = 24;
  arm_mat_init_f32(&ATMAI, srcRows, srcColumns, ATMAI_f32);

  /* Initialise B Matrix Instance with numRows, numCols and data array(B_f32) */
  srcRows = 24;
  srcColumns = 1;
  arm_mat_init_f32(&B, srcRows, srcColumns, (float32_t *)B_f32);

  /* Initialise X Matrix Instance with numRows, numCols and data array(X_f32) */
  srcRows = 24;
  srcColumns = 1;
  arm_mat_init_f32(&X, srcRows, srcColumns, X_f32);

  /* Initialise Kalman gain Matrix K Instance with numRows, numCols and data array(X_f32) */
  //srcRows = 24;
  //srcColumns = 1;
  //arm_mat_init_f32(&K, srcRows, srcColumns, X_f32);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_SET);
	  /* calculation of A transpose */
	  //status = arm_mat_trans_f32(&A, &AT);
	  /* calculation of A inverse */
	  //status = arm_mat_inverse_f32(&A, &AI);
	  /* calculation of AT Multiply with A */
	  //status = arm_mat_mult_f32(&AT, &A, &ATMA);
	  /* calculation of Inverse((Transpose(A) * A) */
	  //status = arm_mat_inverse_f32(&AT, &ATMAI);
	  /* calculation of (Inverse((Transpose(A) * A)) *  Transpose(A)) */
	  //status = arm_mat_mult_f32(&ATMAI, &AT, &ATMA);
	  /* calculation ((Inverse((Transpose(A) * A)) *  Transpose(A)) * B) */
	  //status = arm_mat_mult_f32(&ATMA, &B, &X);

	  /* Perform matrix home-made multiplication */
	      for (c = 0; c < m; c++) {
	        for (d = 0; d < n; d++) {
	          for (k = 0; k < m; k++) {
	            sum = sum + m1[c][k]*m2[k][d];
	          }

	          multiply[c][d] = sum;
	          sum = 0;
	        }
	      }


	  /* Perform transpose matrix */
	  /*for (i = 0; i < m; ++i) {
	          for (j = 0; j < n; ++j) {
	              trans[j][i] = m1[i][j];
	     }
	  }*/
	  /* Perform inverse matrix */
	  /*for(i = 0; i < n; i++){
	          for(j = n; j < 2*n; j++){
	              if(i==(j-n))
	                  m1[i][j] = 1.0;
	              else
	                  m1[i][j] = 0.0;
	          }
	      }

	  for(i = 0; i < n; i++){
	          for(j = 0; j < n; j++){
	              if(i!=j){
	                  ratio = m1[j][i]/m1[i][i];
	                  for(k = 0; k < 2*n; k++){
	                      m1[j][k] -= ratio * m1[i][k];
	                  }
	              }
	          }
	      }

	  for(i = 0; i < n; i++){
		  a = m1[i][i];
		  for(j = 0; j < 2*n; j++){
			  m1[i][j] /= a;
		  }
	  }*/




	  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);
	  HAL_Delay(10);

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
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
