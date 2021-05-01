#ifndef _VOLTAGE_LIB_H_
#define _VOLTAGE_LIB_H_

#include "stm32f4xx_hal.h"
//#include "cmsis_os.h"

class BatVoltage
{
	private:
		ADC_HandleTypeDef* hadc;
		uint16_t adc_raw = 0;
		float V_ref = 0;
		float voltage_divider_ratio = 0;
		float dt = 0.01;
		float k_lp = 4;
		float lpFilterOutput = 2210;
		float filter_adc = 0;
		float error = 0;
		void lpfilter();

	public:
		BatVoltage(ADC_HandleTypeDef* hadc1, float V_ref);
		void getADCdata();
		float getBatVoltage();
		float getRawBatVoltage();
};
#endif
