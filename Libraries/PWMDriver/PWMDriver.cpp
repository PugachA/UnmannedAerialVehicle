#include "PWMDriver.h"

PWMDriver::PWMDriver(TIM_TypeDef *TIM, uint8_t channel, uint32_t min_PWM_value, uint32_t max_PWM_value, double min_mapping_scale, double max_mapping_scale)
{
	this->TIM = TIM;
	this->channel = channel;
	this->min_PWM_value = min_PWM_value;
	this->max_PWM_value = max_PWM_value;
	this->min_mapping_scale = min_mapping_scale;
	this->max_mapping_scale = max_mapping_scale;
	this->multiplier = (max_PWM_value - min_PWM_value) / (max_mapping_scale - min_mapping_scale);
	this->offset = min_PWM_value - this->multiplier * min_mapping_scale;
}

void PWMDriver::setPositionMicroSeconds(uint32_t microseconds)
{
	switch (this->channel)
	{
		case 1:
			this->TIM->CCR1 = microseconds;
			break;
		case 2:
			this->TIM->CCR2 = microseconds;
			break;
		case 3:
			this->TIM->CCR3 = microseconds;
			break;
		case 4:
			this->TIM->CCR4 = microseconds;
			break;
	}
}

void PWMDriver::setPosition(double position)
{
	if(position > this->max_mapping_scale)
		position = this->max_mapping_scale;

	if(position < this->min_mapping_scale)
			position = this->min_mapping_scale;

	uint32_t pwm = (uint32_t)(position * this->multiplier + this->offset);

	this->setPositionMicroSeconds(pwm);
}
