#include "stm32l4xx_hal.h"
#include "MS5611.h"
#include "math.h"

MS5611::MS5611(uint8_t ms5611_addr) //constructor
{
	MS5611_addr = ms5611_addr;
	D1_OSR = 0x48;
	D2_OSR = 0x58;
	ADC_READ = 0x00;
	RST = 0x1E;
	
	timeout = 100;
	comandSize = 1;
	
	R = 8.31;
	T0 = 273.15;
	M = 0.029;
	g = 9.81;
	
	tempDecimation = 100.0;
	presDecimation = 100.0;
}


short MS5611::readProm(unsigned char reg_addr, I2C_HandleTypeDef hi2c1)
{
  const uint16_t bufSize = 2;
  uint8_t buf[bufSize];
  HAL_I2C_Mem_Read(&hi2c1, MS5611_addr << 1, reg_addr, I2C_MEMADD_SIZE_8BIT, buf, bufSize, timeout);
  return (buf[0] << 8) | buf[1];
}

unsigned long MS5611::readBaro(I2C_HandleTypeDef hi2c1)
{
  const uint16_t bufSize = 3;
  uint8_t buf[bufSize];

  HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&D1_OSR, comandSize, timeout); //initiating pressure conversion
  HAL_Delay(10);
  HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&ADC_READ, comandSize, timeout); //initiating ADC reading
  HAL_I2C_Master_Receive(&hi2c1, MS5611_addr << 1, buf, bufSize, timeout);
	
  return (buf[0] << 16) | (buf[1] << 8) | buf[2];
}

unsigned long MS5611::readTemp(I2C_HandleTypeDef hi2c1)
{
  const uint16_t bufSize = 3;
  uint8_t buf[bufSize];

  HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&D2_OSR, comandSize, timeout); //initiating temperature conversion
  HAL_Delay(10);
  HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&ADC_READ, comandSize, timeout); //initiating ADC reading
  HAL_I2C_Master_Receive(&hi2c1, MS5611_addr << 1, buf, bufSize, timeout);
	
  return (buf[0] << 16) | (buf[1] << 8) | buf[2];
}


void MS5611::init(I2C_HandleTypeDef hi2c1) 
{
  // Reset
  HAL_I2C_Master_Transmit(&hi2c1, MS5611_addr << 1,&RST, comandSize, timeout);

  HAL_Delay(10);

  // Read calibration data
  ms5611_C1 = readProm(0xA2, hi2c1);
  ms5611_C2 = readProm(0xA4, hi2c1);
  ms5611_C3 = readProm(0xA6, hi2c1);
  ms5611_C4 = readProm(0xA8, hi2c1);
  ms5611_C5 = readProm(0xAA, hi2c1);
  ms5611_C6 = readProm(0xAC, hi2c1);
}


void MS5611::convertRaw(I2C_HandleTypeDef hi2c1) 
{

  unsigned long D1, D2;
  long dT, TEMP;
  long long OFF, SENS, OFF2, SENS2, T2;
	
  // Read pressure data
  D1 = readBaro(hi2c1);


  // Read Temperature data
  D2 = readTemp(hi2c1);

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

  this->pressure = (unsigned long) (((((D1 * SENS) >> 21) - OFF)) >> 15);
  this->temperature = (long)TEMP;

}

double MS5611::getPressure(I2C_HandleTypeDef hi2c1)
{
  convertRaw(hi2c1);
  return this->pressure/presDecimation;
}

double MS5611::getTemperature(I2C_HandleTypeDef hi2c1)
{
  convertRaw(hi2c1);
  return this->temperature/tempDecimation;
}

double MS5611::getAltitude(I2C_HandleTypeDef hi2c1, double QFEpressure)
{
  convertRaw(hi2c1);
  this->altitude = R*(T0+temperature/tempDecimation)*log((pressure/presDecimation)/QFEpressure)/(-M*g);
  return this->altitude; 
}