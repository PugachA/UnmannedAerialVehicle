#ifndef SERVO_H
#define SERVO_H

#include "stm32f4xx_hal.h"

class Servo
{
	private:
		//значение скважности, при котором серво поворачивается на 180
		uint16_t max_PWM_value;

		//значение скважности, при котором серво поворачивается на 0
		uint16_t min_PWM_value;

		//таймер
		TIM_TypeDef* TIM;

		//канал таймера
		uint8_t channel;

		//максимальный угол поворота сервопривода
		uint16_t max_Angle;

	public:
		Servo(TIM_TypeDef *TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value, uint16_t max_Angle);
		Servo(TIM_TypeDef *TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value);
		Servo(TIM_TypeDef *TIM, uint8_t channel);
		//Поварачивает сервопривод на заданный угол
		void Set_Position(uint8_t position);
		void setPositionMicroSeconds(uint32_t position);
};


#endif // SERVO_H
