#include "P3002.h"

P3002::P3002(ADC_HandleTypeDef hadc)
{
	this->hadc = hadc;
	k_adc_to_angle = 0.00772;
	b_adc_to_angle = 5;
}

uint32_t P3002::convertADC(void)
{
	uint32_t adc_raw = 0;

	HAL_ADC_Start(&this->hadc);
	HAL_ADC_PollForConversion(&this->hadc,100);
	
	adc_raw = HAL_ADC_GetValue(&this->hadc);
	
	HAL_ADC_Stop(&this->hadc);

	return adc_raw;
}

uint32_t P3002::getRawData(void)
{
	return convertADC();
}

double P3002::calcAngle(void)
{
	uint32_t adc_raw = 0;
	double angle = 0.0;

	adc_raw = convertADC();
	angle = k_adc_to_angle*adc_raw + b_adc_to_angle;

	return angle;
}

double P3002::getAngle(void)
{
	return calcAngle();
}
