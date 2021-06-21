/*
 * MPU9250.c
 *
 *	MPU9250 driver for STM32 with HAL using SPI for multiple MPUs
 *  Author: Laura Train, based on https://github.com/desertkun/MPU9250
 *  Date of the last update: March 22 2021
 *
 */

#include "MPU9250.h"
#include "spi.h"


/* Activate chip select */
void MPU9250_Activate(uint16_t MPU9250_CS_PIN)
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_RESET);
}


/* Deactivate chip select */
void MPU9250_Deactivate(uint16_t MPU9250_CS_PIN)
{
	HAL_GPIO_WritePin(MPU9250_CS_GPIO, MPU9250_CS_PIN, GPIO_PIN_SET);
}


/* SPI  transmit & receive */
uint8_t SPIx_WriteRead(uint8_t Byte)
{
	uint8_t receivedbyte = 0;
	if(HAL_SPI_TransmitReceive(&MPU9250_SPI,(uint8_t*) &Byte,(uint8_t*) &receivedbyte,1,0x1000)!=HAL_OK)
	{
		return -1;
	}
	else
	{
	}
	return receivedbyte;
}


/* SPI  write */
void MPU_SPI_Write (uint8_t *pBuffer, uint8_t WriteAddr, uint16_t NumByteToWrite, uint16_t MPU9250_CS_PIN)
{
	MPU9250_Activate(MPU9250_CS_PIN);
	SPIx_WriteRead(WriteAddr);
	while(NumByteToWrite>=0x01)
	{
		SPIx_WriteRead(*pBuffer);
		NumByteToWrite--;
		pBuffer++;
	}
	MPU9250_Deactivate(MPU9250_CS_PIN);
}


/* SPI  read */
void MPU_SPI_Read(uint8_t *pBuffer, uint8_t ReadAddr, uint16_t NumByteToRead, uint16_t MPU9250_CS_PIN)
{
	MPU9250_Activate(MPU9250_CS_PIN);
	uint8_t data = ReadAddr | READWRITE_CMD;
	HAL_SPI_Transmit(&MPU9250_SPI, &data, 1, HAL_MAX_DELAY);
	HAL_SPI_Receive(&MPU9250_SPI, pBuffer, NumByteToRead, HAL_MAX_DELAY);
	MPU9250_Deactivate(MPU9250_CS_PIN);
}


/* SPI write a byte to MPU9250 register given a register address and data */
void writeRegister(uint8_t subAddress, uint8_t data, uint16_t MPU9250_CS_PIN)
{
	MPU_SPI_Write(&data, subAddress, 1, MPU9250_CS_PIN);
	HAL_Delay(10);
}


/* SPI read registers from MPU9250 given a starting register address, number of bytes, and a pointer to store data */
void readRegisters(uint8_t subAddress, uint8_t count, uint8_t* dest, uint16_t MPU9250_CS_PIN){
	MPU_SPI_Read(dest, subAddress, count, MPU9250_CS_PIN);
}


/* I2C write a register to the AK8963 given a register address and data */
void writeAK8963Register(uint8_t subAddress, uint8_t data, uint16_t MPU9250_CS_PIN)
{
	// set slave 0 to the AK8963 and set for write
	writeRegister(I2C_SLV0_ADDR,AK8963_I2C_ADDR, MPU9250_CS_PIN);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG,subAddress, MPU9250_CS_PIN);

	// store the data for write
	writeRegister(I2C_SLV0_DO, data, MPU9250_CS_PIN);

	// enable I2C and send 1 byte
	writeRegister(I2C_SLV0_CTRL,I2C_SLV0_EN | (uint8_t)1, MPU9250_CS_PIN);
}


/* I2C read registers from the AK8963 */
void readAK8963Registers(uint8_t subAddress, uint8_t count, uint8_t* dest, uint16_t MPU9250_CS_PIN)
{
	// set slave 0 to the AK8963 and set for read
	writeRegister(I2C_SLV0_ADDR, AK8963_I2C_ADDR | I2C_READ_FLAG, MPU9250_CS_PIN);

	// set the register to the desired AK8963 sub address
	writeRegister(I2C_SLV0_REG, subAddress, MPU9250_CS_PIN);

	// enable I2C and request the bytes
	writeRegister(I2C_SLV0_CTRL, I2C_SLV0_EN | count, MPU9250_CS_PIN);

	// takes some time for these registers to fill
	HAL_Delay(1);

	// read the bytes off the MPU9250 EXT_SENS_DATA registers
	readRegisters(EXT_SENS_DATA_00, count, dest, MPU9250_CS_PIN);
}


/* Get the MPU9250 WHO_AM_I register value, expected to be 0x71 */
uint8_t whoAmI(uint16_t MPU9250_CS_PIN){
	// read the WHO AM I register
	readRegisters(WHO_AM_I, 1, _buffer, MPU9250_CS_PIN);

	// return the register value
	return _buffer[0];
}


/* Get the AK8963 WHO_AM_I register value, expected to be 0x48 */
int whoAmIAK8963(uint16_t MPU9250_CS_PIN){
	// read the WHO AM I register
	readAK8963Registers(AK8963_WHO_AM_I, 1, _buffer, MPU9250_CS_PIN);
	// return the register value
	return _buffer[0];
}


/* Initialize communication with the MPU-9250 */
uint8_t MPU9250_Init(uint16_t MPU9250_CS_PIN)
{
	// activate chip select
	MPU9250_Activate(MPU9250_CS_PIN);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL, MPU9250_CS_PIN);

	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN, MPU9250_CS_PIN);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK, MPU9250_CS_PIN);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN, MPU9250_CS_PIN);

	// reset the MPU9250
	writeRegister(PWR_MGMNT_1, PWR_RESET, MPU9250_CS_PIN);

	// wait for MPU-9250 to come back up
	HAL_Delay(10);
	// reset the AK8963
	writeAK8963Register(AK8963_CNTL2, AK8963_RESET, MPU9250_CS_PIN);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL, MPU9250_CS_PIN);

	// check the WHO AM I byte, expected value is 0x71 (decimal 113) or 0x73 (decimal 115)
	uint8_t who = whoAmI(MPU9250_CS_PIN);
	if((who != 0x71) &&( who != 0x73))
	{
		return 1;
	}

	// enable accelerometer and gyro
	writeRegister(PWR_MGMNT_2, SEN_ENABLE, MPU9250_CS_PIN);

	// setting accel range to 16G as default
	writeRegister(ACCEL_CONFIG, ACCEL_FS_SEL_16G, MPU9250_CS_PIN);

	// setting the gyro range to 2000DPS as default
	writeRegister(GYRO_CONFIG, GYRO_FS_SEL_2000DPS, MPU9250_CS_PIN);

	// setting bandwidth to 184Hz as default
	writeRegister(ACCEL_CONFIG2, DLPF_184, MPU9250_CS_PIN);

	// setting gyro bandwidth to 184Hz
	writeRegister(CONFIG, DLPF_184, MPU9250_CS_PIN);

	// setting the sample rate divider to 0 as default
	writeRegister(SMPDIV, 0x00, MPU9250_CS_PIN);

	// enable I2C master mode
	writeRegister(USER_CTRL, I2C_MST_EN, MPU9250_CS_PIN);

	// set the I2C bus speed to 400 kHz
	writeRegister(I2C_MST_CTRL, I2C_MST_CLK, MPU9250_CS_PIN);

	// check AK8963 WHO AM I register, expected value is 0x48 (decimal 72)
	if( whoAmIAK8963(MPU9250_CS_PIN) != 0x48 )
	{
		return 1;
	}

	/* get the magnetometer calibration */
	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN, MPU9250_CS_PIN);

	HAL_Delay(100); // long wait between AK8963 mode changes

	// set AK8963 to FUSE ROM access
	writeAK8963Register(AK8963_CNTL1, AK8963_FUSE_ROM, MPU9250_CS_PIN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// read the AK8963 ASA registers and compute magnetometer scale factors
	readAK8963Registers(AK8963_ASA, 3, _mag_adjust, MPU9250_CS_PIN);

	// set AK8963 to Power Down
	writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN, MPU9250_CS_PIN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// set AK8963 to 16 bit resolution, 100 Hz update rate
	writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2, MPU9250_CS_PIN);

	// long wait between AK8963 mode changes
	HAL_Delay(100);

	// select clock source to gyro
	writeRegister(PWR_MGMNT_1, CLOCK_SEL_PLL, MPU9250_CS_PIN);

	// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
	readAK8963Registers(AK8963_HXL,7,_buffer, MPU9250_CS_PIN);

	// deactivate chip select
	MPU9250_Deactivate(MPU9250_CS_PIN);

	// successful init, return 0
	return 0;
}


/* Set the accelerometer full scale range to values other than default */
void MPU9250_SetAccelRange(AccelRange range, uint16_t MPU9250_CS_PIN)
{
	writeRegister(ACCEL_CONFIG, range, MPU9250_CS_PIN);
}


/* Set the gyro full scale range to values other than default */
void MPU9250_SetGyroRange(GyroRange range, uint16_t MPU9250_CS_PIN)
{
	writeRegister(GYRO_CONFIG, range, MPU9250_CS_PIN);
}


/* Set the DLPF bandwidth to values other than default */
void MPU9250_SetDLPFBandwidth(DLPFBandwidth bandwidth, uint16_t MPU9250_CS_PIN)
{
	writeRegister(ACCEL_CONFIG2, bandwidth, MPU9250_CS_PIN);
	writeRegister(CONFIG, bandwidth, MPU9250_CS_PIN);
}


/* Set the sample rate divider to values other than default */
void MPU9250_SetSampleRateDivider(SampleRateDivider srd, uint16_t MPU9250_CS_PIN){

	/* setting the sample rate divider to 19 to facilitate setting up magnetometer */
	writeRegister(SMPDIV, 19, MPU9250_CS_PIN);

	if(srd > 9)
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN, MPU9250_CS_PIN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// set AK8963 to 16 bit resolution, 8 Hz update rate
		writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS1, MPU9250_CS_PIN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL, 7, _buffer, MPU9250_CS_PIN);

	}
	else
	{
		// set AK8963 to Power Down
		writeAK8963Register(AK8963_CNTL1, AK8963_PWR_DOWN, MPU9250_CS_PIN);
		// long wait between AK8963 mode changes
		HAL_Delay(100);
		// set AK8963 to 16 bit resolution, 100 Hz update rate
		writeAK8963Register(AK8963_CNTL1, AK8963_CNT_MEAS2, MPU9250_CS_PIN);

		// long wait between AK8963 mode changes
		HAL_Delay(100);

		// instruct the MPU9250 to get 7 bytes of data from the AK8963 at the sample rate
		readAK8963Registers(AK8963_HXL, 7, _buffer, MPU9250_CS_PIN);
	}

	writeRegister(SMPDIV, srd, MPU9250_CS_PIN);
}



/* Read the raw data from accelerometer, gyroscope and magnetometer */
void MPU9250_GetData(int16_t AccData[], int16_t GyroData[], int16_t MagData[], uint16_t MPU9250_CS_PIN)
{
	// activate chip select
	MPU9250_Activate(MPU9250_CS_PIN);

	// grab the data from the MPU9250
	readRegisters(ACCEL_OUT, 21, _buffer, MPU9250_CS_PIN);

	int16_t magx, magy, magz;

	// combine into 16 bit values
	AccData[0] = (((int16_t)_buffer[0]) << 8) | _buffer[1];
	AccData[1] = (((int16_t)_buffer[2]) << 8) | _buffer[3];
	AccData[2] = (((int16_t)_buffer[4]) << 8) | _buffer[5];

	GyroData[0] = (((int16_t)_buffer[8]) << 8) | _buffer[9];
	GyroData[1] = (((int16_t)_buffer[10]) << 8) | _buffer[11];
	GyroData[2] = (((int16_t)_buffer[12]) << 8) | _buffer[13];

	magx = (((int16_t)_buffer[15]) << 8) | _buffer[14];
	magy = (((int16_t)_buffer[17]) << 8) | _buffer[16];
	magz = (((int16_t)_buffer[19]) << 8) | _buffer[18];

	MagData[0] = ((int16_t)magx * ((float)(_mag_adjust[0] - 128) / 256.0f + 1.0f));
	MagData[1] = ((int16_t)magy * ((float)(_mag_adjust[1] - 128) / 256.0f + 1.0f));
	MagData[2] = ((int16_t)magz * ((float)(_mag_adjust[2] - 128) / 256.0f + 1.0f));

	// deactivate chip select
	MPU9250_Deactivate(MPU9250_CS_PIN);
}


