#include "MPXV7002.h"
#include "math.h"

MPXV7002::MPXV7002(ADC_HandleTypeDef hadc)
{
	this->hadc = hadc;
	rho = 1.225; // kg/m^3
	k_adc_to_pressure = 0.001221;
	b_adc_to_pressure = 0; //old = -2.5
	adc_offset = 0.0;
	this->filtered_adc = 0;
}

void MPXV7002::convertADC(void)
{
	HAL_ADC_Start(&this->hadc);
	HAL_ADC_PollForConversion(&this->hadc,100);

	this->adc_raw = HAL_ADC_GetValue(&this->hadc);
	
	HAL_ADC_Stop(&this->hadc);
}
void MPXV7002::calibrateZeroADC()
{
	for(int i=0; i<20; i++)
	{
		convertADC();
		adc_offset += adc_raw;
		HAL_Delay(50);
	}
	adc_offset /= 20;
}
void MPXV7002::filterADC(void)
{
	convertADC();
	this->filtered_adc = (1 - 0.1) *this->filtered_adc + 0.1*((double)this->adc_raw - adc_offset);
}
void MPXV7002::calcPressure(void)
{
	filterADC();
	this->pressure = 1000*(k_adc_to_pressure*double(this->filtered_adc)); //1000 converts kPa to Pa
}
void MPXV7002::calcAirSpeed(void)
{
	calcPressure();
	this->airSpeed = sqrt(2*abs(this->pressure)/rho);
}
uint32_t MPXV7002::getRawData(void)
{
	convertADC();
	return this->adc_raw;
}
uint32_t MPXV7002::getFilteredADC(void)
{
	filterADC();
	return this->filtered_adc;
}
double MPXV7002::getPressure(void)
{
	calcPressure();
	return this->pressure;
}
double MPXV7002::getAirSpeed(void)
{
	calcAirSpeed();
	return this->airSpeed;
}


