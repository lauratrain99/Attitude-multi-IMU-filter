/*
 * MPU9250_Config.h
 *
 *	MPU9250 driver for STM32 with HAL using SPI for multiple MPUs
 *  Author: Laura Train, based on https://github.com/desertkun/MPU9250
 *  Date of the last update: March 22 2021
 *
 */

/* SPI communication protocol */
#define MPU9250_SPI		      	    hspi2

/* GPIO port */
#define	MPU9250_CS_GPIO		        GPIOA

/* GPIO chip select pins for each IMU*/
#define	MPU9250_1_CS_PIN	        GPIO_PIN_6
#define	MPU9250_2_CS_PIN		    GPIO_PIN_7
#define	MPU9250_3_CS_PIN	    	GPIO_PIN_8
#define	MPU9250_4_CS_PIN	    	GPIO_PIN_9
