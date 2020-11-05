#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

class Servo
{
	private:
		//максимальное рабочая скважность
		uint32_t max_PWM_value;
		//минимальная рабочая скважность
		uint32_t min_PWM_value;
		double min_input;
		double max_input;
		double multiplier;
		//таймер
		TIM_TypeDef* TIM;
		//канал таймера
		uint8_t channel;

	public:
		Servo(TIM_TypeDef *TIM, uint8_t channel, uint32_t min_PWM_value, uint32_t max_PWM_value, double min_input, double max_input);
		Servo(TIM_TypeDef *TIM, uint8_t channel, uint32_t min_PWM_value, uint32_t max_PWM_value);

		void setPosition(double position);
		void setPositionMicroSeconds(uint32_t microseconds);
};


#endif // SERVO_H
