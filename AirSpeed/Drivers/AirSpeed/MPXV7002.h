#ifndef _MPXV7002_LIB_H_
#define _MPXV7002_LIB_H_

#include "stm32l4xx_hal.h"
class MPXV7002
{
  private:
		ADC_HandleTypeDef hadc; //pointer to ADC
	
	  uint32_t adc_raw;
	  double pressure;
	  double airSpeed;
	
	  void convertADC(void);
	  void calcPressure(void);
	  void calcAirSpeed(void);
	
	  //physical constants
    double rho; //air density
	
	  // caharacteristics coefficients
	  double k_adc_to_pressure;
	  double b_adc_to_pressure; // pressure = k*adc_raw + b
	  
	  		
	
  public:
		MPXV7002(ADC_HandleTypeDef);//constructor
	  uint32_t getRawData(void);
	  double getPressure(void);
	  double getAirSpeed(void);
};
#endif
