#include "BatVoltage.h"

BatVoltage::BatVoltage(ADC_HandleTypeDef* hadc1, float V_ref)
{
	this->hadc = hadc1;
	this->V_ref = V_ref;
	this->voltage_divider_ratio = 29/3;
}
void BatVoltage::getADCdata()
{
	adc_raw = HAL_ADC_GetValue(hadc);
}

float BatVoltage::getBatVoltage()
{
	HAL_ADC_Start_IT(hadc);
	lpfilter();
	return (V_ref/4096 * filter_adc * voltage_divider_ratio)+0.7;
}

float BatVoltage::getRawBatVoltage()
{
	return (V_ref/4096 * (float)adc_raw * voltage_divider_ratio)+0.7;
}

void BatVoltage::lpfilter()
{
  	error = k_lp*((float)adc_raw - lpFilterOutput);
  	lpFilterOutput += error*dt;
  	filter_adc = lpFilterOutput;
}
