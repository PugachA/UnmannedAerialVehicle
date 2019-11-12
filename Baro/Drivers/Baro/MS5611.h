#ifndef _MS5611_LIB_H_
#define _MS5611_LIB_H_

#include "stm32l4xx_hal.h"

class MS5611
{
  private:
    uint8_t MS5611_addr;
    I2C_HandleTypeDef hi2c;
	
    double pressure_QFE;
	  
    //constants for raw data reading
    uint8_t D1_OSR;
    uint8_t D2_OSR;
    uint8_t ADC_READ;
    uint8_t RST;
	
    //constants for HAL functions
    uint32_t timeout;
    uint16_t comandSize;
	  
    //calibration coefficients
    unsigned short ms5611_C1, ms5611_C2, ms5611_C3, ms5611_C4, ms5611_C5, ms5611_C6;
	
    //physical constants for altitude computing
    double R; //gas constatnt
    double T0; //difference between Kelvin and Celcius
    double M; //air molar mass
    double g; //acceleration of gravity
	
    //decimation coefficients
    double tempDecimation;
    double presDecimation;
	
    long temperature;
    long pressure;
    double altitude;
	
    short readProm(unsigned char); //read one of calibration coeffecients 
    unsigned long readBaro(void); //read raw pressure
    unsigned long readTemp(void); //read raw temperature
    void convertRaw(void); //convert pressure and temp from raw
		
		
	
  public:
    MS5611(uint8_t,I2C_HandleTypeDef); //constructor

    void init(void); //reset and init all calibration coefficients
	
    double getPressure(void); //return pressure
    double getTemperature(void); //return temperature
    double getAltitude(void); //return altitude depending on pressure on ground
		
			
};
#endif