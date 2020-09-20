#include "MPXV7002.h"
#include "math.h"

MPXV7002::MPXV7002(ADC_HandleTypeDef hadc)
{
	this->hadc = hadc;
	rho = 1.225; // kg/m^3
	k_adc_to_pressure = 0.001221;
	b_adc_to_pressure = -2.5;
}

void MPXV7002::convertADC(void)
{
	HAL_ADC_Start(&this->hadc);
	HAL_ADC_PollForConversion(&this->hadc,100);
	
	this->adc_raw = HAL_ADC_GetValue(&this->hadc);
	
	HAL_ADC_Stop(&this->hadc);
}

uint32_t MPXV7002::getRawData(void)
{
	convertADC();
	return this->adc_raw;
}

void MPXV7002::calcPressure(void)
{
	convertADC();
	this->pressure = 1000*(k_adc_to_pressure*double(this->adc_raw) + b_adc_to_pressure); //1000 converts kPa to Pa
}

double MPXV7002::getPressure(void)
{
	calcPressure();
	return this->pressure;
}

void MPXV7002::calcAirSpeed(void)
{
	calcPressure();
	this->airSpeed = sqrt(2*abs(this->pressure)/rho);
}

double MPXV7002::getAirSpeed(void)
{
	calcAirSpeed();
	return this->airSpeed;
}

