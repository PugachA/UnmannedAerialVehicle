#ifndef _MS5611_LIB_H_
#define _MS5611_LIB_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

class MS5611
{
  private:
    uint8_t MS5611_addr; //address of the sensor on the bus
    I2C_HandleTypeDef *hi2c; //pointer to the i2c bus
	
    double pressure_QFE; //used in getAltitude to calc altitude AGL
	
    int points_to_average; //used in updateQFE to calc average pressure
	  
    //constants for raw data reading
    uint8_t D1_OSR;
    uint8_t D2_OSR;
    uint8_t ADC_READ;
    uint8_t RST;
	
    //constants for HAL functions
    uint32_t timeout;
    uint16_t comandSize;
	  
    //calibration coefficients
    uint16_t ms5611_C1, ms5611_C2, ms5611_C3, ms5611_C4, ms5611_C5, ms5611_C6;
	
    //physical constants for altitude computing
    double R; //gas constatnt
    double T0; //difference between Kelvin and Celcius
    double M; //air molar mass
    double g; //acceleration of gravity
	
    //decimation coefficients
    double temp_decimation;
    double pres_decimation;
	
    int32_t temperature;
    int32_t pressure;
    double rawAltitude;
    double filterAltitude;

    double dt; //period between timer interrupts for integrals
	
    short readProm(unsigned char); //read one of calibration coeffecients 
    uint32_t readBaro(void); //read raw pressure
    uint32_t readTemp(void); //read raw temperature
    void convertRaw(void); //convert pressure and temp from raw

	void lpAltFilter(); //low pass filter for raw altitude
	double lpFilterOutput;
	double k_lp_alt; //coefficient for altitude low pass filter
		
	//Vy calc
	double first_filter_output = 0;
	double second_filter_output = 0;
	double third_filter_output = 0;
	double fourth_filter_output = 0;
	double k1; //coefficient in the first filter for Vy calculation
	double k2; //coefficient in the second filter for Vy calculation
	double k3; //coefficient in the third filter for Vy calculation
	double k4; //coefficient in the foutrth filter for Vy calculation
	double vertical_speed;
		
	void firstVsFilter();
	void secondVsFilter();
	void thirdVsFilter();
	void fourthVsFilter();
		
		
	
  public:
    MS5611(uint8_t ms5611_addr, I2C_HandleTypeDef *hi2c,int number_of_points_to_average, double dt); //constructor
    void calcAltitude(void); //calculating altitude depending on pressure on ground
    double getPressure(void); //return pressure
    double getTemperature(void); //return temperature
    double getRawAltitude(void); //return raw altitude
    double getFilterAltitude(void); //return filtered altitude
    void updateQFE(void); //calculating and remembering QFE pressure, public in case of need to re-init QFE from main
    double getQFEpressure(void); //return rememberred QFE pressure
    void calcVerticalSpeed(void); //calc Vy in timer interrupt
    double getVerticalSpeed(void); //return Vy
		
			
};
#endif
