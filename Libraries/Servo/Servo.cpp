#include "Servo.h"

Servo::Servo(TIM_TypeDef *TIM, uint8_t channel, uint32_t min_PWM_value, uint32_t max_PWM_value, double min_input, double max_input)
{
	this->TIM = TIM;
	this->channel = channel;
	this->min_PWM_value = min_PWM_value;
	this->max_PWM_value = max_PWM_value;
	this->min_input = min_input;
	this->max_input = max_input;
	this->multiplier = (max_PWM_value - min_PWM_value) / (max_input - min_input);
}

Servo::Servo(TIM_TypeDef *TIM, uint8_t channel, uint32_t min_PWM_value, uint32_t max_PWM_value)
	: Servo(TIM, channel, min_PWM_value, max_PWM_value, 0, 100)
{}

void Servo::setPositionMicroSeconds(uint32_t microseconds)
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

void Servo::setPosition(double position)
{
	if(position > this->max_input)
		position = this->max_input;

	uint32_t pwm = this->min_PWM_value + (uint32_t)((position - this->min_input) * this->multiplier);

	this->setPositionMicroSeconds(pwm);
}
