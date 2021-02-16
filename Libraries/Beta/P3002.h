#ifndef _P3002_LIB_H_
#define _P3002_LIB_H_

#include "stm32f4xx_hal.h"
class P3002
{
  private:
	  ADC_HandleTypeDef *hadc; //pointer to ADC
	
	  uint32_t convertADC(void);
	  double calcAngle(void);
	
	  // characteristics coefficients
	  double k_adc_to_angle;
	  double b_adc_to_angle; // angle = k*adc_raw + b
	  
	  		
	
  public:
	  P3002(ADC_HandleTypeDef *hadc);//constructor
	  uint32_t getRawData(void);
	  double getAngle(void);
};
#endif
