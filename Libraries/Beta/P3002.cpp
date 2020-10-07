#include "P3002.h"
#include "math.h"

P3002::P3002(ADC_HandleTypeDef hadc)
{
	this->hadc = hadc;
	k_adc_to_angle = 0.001221;
	b_adc_to_angle = -2.5;
}

void P3002::convertADC(void)
{
	HAL_ADC_Start(&this->hadc);
	HAL_ADC_PollForConversion(&this->hadc,100);
	
	this->adc_raw = HAL_ADC_GetValue(&this->hadc);
	
	HAL_ADC_Stop(&this->hadc);
}

uint32_t P3002::getRawData(void)
{
	convertADC();
	return this->adc_raw;
}

void P3002::calcAngle(void)
{
	convertADC();
	this->angle = k_adc_to_angle*double(this->adc_raw) + b_adc_to_angle;
}

double P3002::getAngle(void)
{
	calcAngle();
	return this->angle;
}
