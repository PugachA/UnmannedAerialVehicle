#ifndef PWMDriver_H
#define PWMDriver_H

#include "stm32f4xx_hal.h"

class PWMDriver
{
	private:
		//максимальное рабочая скважность
		uint32_t max_PWM_value;
		//минимальная рабочая скважность
		uint32_t min_PWM_value;
		double min_mapping_scale;
		double max_mapping_scale;
		double multiplier;
		double offset;
		//таймер
		TIM_TypeDef* TIM;
		//канал таймера
		uint8_t channel;

	public:
		PWMDriver(TIM_TypeDef *TIM,
				uint8_t channel,
				uint32_t min_PWM_value,
				uint32_t max_PWM_value,
				double min_mapping_scale,
				double max_mapping_scale);

		void setPosition(double position);
		void setPositionMicroSeconds(uint32_t microseconds);
};


#endif // PWMDriver_H
