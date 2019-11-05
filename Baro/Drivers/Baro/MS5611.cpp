#include "stm32l4xx_hal.h"
#include "MS5611.h"
#include "math.h"

MS5611::MS5611(uint8_t ms5611_addr) //constructor
{
	MS5611_addr = ms5611_addr;
}

//initialising ststic variables with data that should be put into ms5611 registers
uint8_t MS5611::D1_OSR = 0x48;
uint8_t MS5611::D2_OSR = 0x58;
uint8_t MS5611::ADC_READ = 0x00;
uint8_t MS5611::RST = 0x1E;

short MS5611::ms5611ReadPROM(unsigned char reg_addr, I2C_HandleTypeDef hi2c1)
{
  uint8_t buf[2];
	HAL_I2C_Mem_Read(&hi2c1, MS5611_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buf, 2, 100);
  return (buf[0] << 8) | buf[1];
}

unsigned long MS5611::ms5611ReadBARO(I2C_HandleTypeDef hi2c1)
{
	uint8_t buf[3];

	HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&D1_OSR, 1, 100); //initiating pressure conversion
	HAL_Delay(10);
  HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&ADC_READ, 1, 100); //initiating ADC reading
	HAL_I2C_Master_Receive(&hi2c1, MS5611_addr << 1, buf, 3, 100);
	
  return (buf[0] << 16) | (buf[1] << 8) | buf[2];
}

unsigned long MS5611::ms5611ReadTEMP(I2C_HandleTypeDef hi2c1)
{
  uint8_t buf[3];

	HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&D2_OSR, 1, 100); //initiating temperature conversion
	HAL_Delay(10);
  HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&ADC_READ, 1, 100); //initiating ADC reading
	HAL_I2C_Master_Receive(&hi2c1, MS5611_addr << 1, buf, 3, 100);
	
  return (buf[0] << 16) | (buf[1] << 8) | buf[2];
}


void MS5611::ms5611_Init(I2C_HandleTypeDef hi2c1) 
{
	// Reset
	HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&RST, 1, 100);

	HAL_Delay(10);

	// Read calibration data
	ms5611_C1 = ms5611ReadPROM(0xA2, hi2c1);
	ms5611_C2 = ms5611ReadPROM(0xA4, hi2c1);
	ms5611_C3 = ms5611ReadPROM(0xA6, hi2c1);
	ms5611_C4 = ms5611ReadPROM(0xA8, hi2c1);
	ms5611_C5 = ms5611ReadPROM(0xAA, hi2c1);
	ms5611_C6 = ms5611ReadPROM(0xAC, hi2c1);
}

void MS5611::ms5611_Convert(long* temperature, long* pressure, I2C_HandleTypeDef hi2c1) 
{

	unsigned long D1, D2;
	long dT, TEMP;
	long long OFF, SENS, OFF2, SENS2, T2;
	
	// Read pressure data
	D1 = ms5611ReadBARO(hi2c1);


	// Read Temperature data
	D2 = ms5611ReadTEMP(hi2c1);

	dT = D2 - (ms5611_C5 << 8);
	TEMP = 2000 + (((long long)dT * (long long)ms5611_C6) >> 23);
	OFF = ((long long)ms5611_C2 << 16) + (((long long)ms5611_C4 * (long long)dT) >> 7);
	SENS = ((long long)ms5611_C1 << 15 ) + (((long long)ms5611_C3 * (long long)dT ) >> 8);

	if (TEMP >= 2000) {
		T2 = 0;
		OFF2 = 0;
		SENS2 = 0;
	}
	else if (TEMP < 2000) {
		T2 = (((long long)dT * (long long)dT) >> 31);
		OFF2 = 5 * (((long long)TEMP - 2000) * ((long long)TEMP - 2000)) >> 1;
		SENS2 = 5 * (((long long)TEMP - 2000) * ((long long)TEMP - 2000)) >> 2;
		if (TEMP < -1500 ) {
			OFF2 = OFF2 + 7 * (((long long)TEMP + 1500) * ((long long)TEMP + 1500));
			SENS2 = SENS2 + ((11 *(((long long)TEMP + 1500) * ((long long)TEMP + 1500))) >> 1);
		}
	}

	TEMP = TEMP - T2;
	OFF = OFF - OFF2;
	SENS = SENS - SENS2;

	*pressure = (unsigned long) (((((D1 * SENS) >> 21) - OFF)) >> 15);
	*temperature = (long)TEMP;

}