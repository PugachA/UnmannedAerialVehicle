#include "Servo.h"

Servo::Servo(TIM_TypeDef* TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value, uint16_t max_Angel)
{
	this->TIM = TIM;
	this->channel = channel;
	this->min_PWM_value = min_PWM_value;
	this->max_PWM_value = max_PWM_value;
	this->max_Angel = max_Angel;
}

Servo::Servo(TIM_TypeDef* TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value)
{
	this->TIM = TIM;
	this->channel = channel;
	this->min_PWM_value = min_PWM_value;
	this->max_PWM_value = max_PWM_value;
	this->max_Angel = 180;
}

void  Servo::Set_Position(uint8_t position)
{
  double multiplier = (double)(this->max_PWM_value - this->min_PWM_value)/this->max_Angel ;
	
	if(position > this->max_Angel)
		position = this->max_Angel;
	
	uint16_t pwm = min_PWM_value + multiplier * position;
	
	switch (this->channel)
	{
		case 1:
			this->TIM->CCR1 = pwm;
			break;
		case 2:
			this->TIM->CCR2 = pwm;
			break;
		case 3:
			this->TIM->CCR3 = pwm;
			break;
		case 4:
			this->TIM->CCR4 = pwm;
			break;
		case 5:
			this->TIM->CCR5 = pwm;
			break;
		case 6:
			this->TIM->CCR6 = pwm;
			break;
		default:
			this->TIM->CCR1 = pwm;
			break;
	}
}