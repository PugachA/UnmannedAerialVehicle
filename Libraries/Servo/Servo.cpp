#include "Servo.h"

Servo::Servo(TIM_HandleTypeDef *TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value, uint16_t max_Angle)
{
	this->TIM = TIM;
	this->channel = channel;
	this->min_PWM_value = min_PWM_value;
	this->max_PWM_value = max_PWM_value;
	this->max_Angle = max_Angle;
}

Servo::Servo(TIM_HandleTypeDef *TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value)
	: Servo(TIM, channel, min_PWM_value, max_PWM_value, 180)
{}

Servo::Servo(TIM_HandleTypeDef *TIM, uint8_t channel)
	: Servo(TIM, channel, 0, 0, 0)
{}

void Servo::setPositionMicroSeconds(uint32_t position)
{
	switch (this->channel)
	{
		case 1:
			this->TIM->Instance->CCR1 = position;
			break;
		case 2:
			this->TIM->Instance->CCR2 = position;
			break;
		case 3:
			this->TIM->Instance->CCR3 = position;
			break;
		case 4:
			this->TIM->Instance->CCR4 = position;
			break;
	}
}

void Servo::Set_Position(uint8_t position)
{
	double multiplier = (double)(this->max_PWM_value - this->min_PWM_value)/this->max_Angle;

	if(position > this->max_Angle)
		position = this->max_Angle;

	uint16_t pwm = min_PWM_value + multiplier * position;

	switch (this->channel)
	{
		case 1:
			this->TIM->Instance->CCR1 = pwm;
			break;
		case 2:
			this->TIM->Instance->CCR2 = pwm;
			break;
		case 3:
			this->TIM->Instance->CCR3 = pwm;
			break;
		case 4:
			this->TIM->Instance->CCR4 = pwm;
			break;
	}
}
