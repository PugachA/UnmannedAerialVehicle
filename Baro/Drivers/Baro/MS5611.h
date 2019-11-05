#ifndef _MS5611_LIB_H_
#define _MS5611_LIB_H_

#include "stm32l4xx_hal.h"

class MS5611
{
	private:
		static uint8_t D1_OSR;
		static uint8_t D2_OSR;
		static uint8_t ADC_READ;
		static uint8_t RST;
		unsigned short ms5611_C1, ms5611_C2, ms5611_C3, ms5611_C4, ms5611_C5, ms5611_C6;
	
		short ms5611ReadPROM(unsigned char, I2C_HandleTypeDef); //read one of calibration coeffecients 
		unsigned long ms5611ReadBARO(I2C_HandleTypeDef); //read raw pressure
		unsigned long ms5611ReadTEMP(I2C_HandleTypeDef); //read raw temperature
		
		
	
	public:
		uint8_t MS5611_addr;
		MS5611(uint8_t); //constructor
	
		void ms5611_Init(I2C_HandleTypeDef); //reset and init all calibration coefficients
		void ms5611_Convert(long*, long*, I2C_HandleTypeDef); //convert pressure and temp from raw
		
			
};

#endif