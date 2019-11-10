#include "stm32f3xx_hal.h"

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
		uint16_t max_Angel; 
	
	public:
		Servo(TIM_TypeDef* TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value, uint16_t max_Angel);
		Servo(TIM_TypeDef* TIM, uint8_t channel, uint16_t min_PWM_value, uint16_t max_PWM_value);
		//Поварачивает сервопривод на заданный угол
		void Set_Position(uint8_t position);
};
