/*
 * PWMCapturer.cpp
 *
 *  Created on: Oct 10, 2020
 *      Author: foton
 */

#include "PWMCapturer/PWMCapturer.h"

PWMCapturer::PWMCapturer(TIM_HandleTypeDef *htim,
		uint8_t channel,
		uint16_t min_value,
		uint16_t mid_value,
		uint16_t max_value,
		uint8_t measurement_error)
{
	this->min_value = min_value;
	this->max_value = max_value;
	this->mid_value = mid_value;
	this->measurement_error = measurement_error;

	this->htim = htim;
	switch(channel)
	{
		case 1:
			this->channel = TIM_CHANNEL_1;
			break;
		case 2:
			this->channel = TIM_CHANNEL_2;
			break;
		case 3:
			this->channel = TIM_CHANNEL_3;
			break;
		case 4:
			this->channel = TIM_CHANNEL_4;
			break;
	}
}

PWMCapturer::PWMCapturer(TIM_HandleTypeDef *htim,
		uint8_t channel,
		uint16_t min_value,
		uint16_t max_value,
		uint8_t measurement_error)
	: PWMCapturer(htim, channel, min_value, min_value + (max_value - min_value)/2, max_value, measurement_error)
{}

PWMCapturer::~PWMCapturer() {
	// TODO Auto-generated destructor stub
}

uint32_t PWMCapturer::getPulseWidth()
{
	return difference;
}

uint32_t PWMCapturer::getMaxWidth()
{
	return max_value;
}

uint32_t PWMCapturer::getMinWidth()
{
	return min_value;
}

uint32_t PWMCapturer::getMidWidth()
{
	return mid_value;
}

void PWMCapturer::calculatePulseWidth()
{
	if(!is_First_Captured) //if the first value is not captured
	{
		IC_Val1 = HAL_TIM_ReadCapturedValue(htim, channel); //read the first value
		is_First_Captured = true;  //set the first captured as true
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_FALLING); //Now change the polarity to falling edge
	}
	else //if the first is already captured
	{
		IC_Val2 = HAL_TIM_ReadCapturedValue(htim, channel);  //read second value
		//__HAL_TIM_SET_COUNTER(htim, 0);  //reset the counter

		if (IC_Val2 > IC_Val1)
			difference = IC_Val2 - IC_Val1;

		is_First_Captured = false; //set it back to false
		__HAL_TIM_SET_CAPTUREPOLARITY(htim, channel, TIM_INPUTCHANNELPOLARITY_RISING); //set polarity to rising edge
	}
}

uint32_t PWMCapturer::getPulseWidthDif()
{
	return mid_value - (difference - mid_value);
}

bool PWMCapturer::matchMinValue()
{
	if((difference > (min_value - measurement_error)) && (difference < (min_value + measurement_error)))
		return true;

	return false;
}

bool PWMCapturer::matchMidValue()
{
	if((difference > (mid_value - measurement_error)) && (difference < (mid_value + measurement_error)))
		return true;

	return false;
}

bool PWMCapturer::matchMaxValue()
{
	if((difference > (max_value - measurement_error)) && (difference < (max_value + measurement_error)))
		return true;

	return false;
}

