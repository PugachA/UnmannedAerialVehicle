/*
 * PWMCapturer.h
 *
 *  Created on: Oct 10, 2020
 *      Author: foton
 */

#ifndef PWMCAPTURER_PWMCAPTURER_H_
#define PWMCAPTURER_PWMCAPTURER_H_

#include "stm32f4xx_hal.h"

class PWMCapturer
{
	public:
		PWMCapturer(TIM_HandleTypeDef *htim,
				uint8_t channel,
				uint16_t min_value,
				uint16_t mid_value,
				uint16_t max_value,
				uint8_t measurement_error);
		PWMCapturer(TIM_HandleTypeDef *htim,
				uint8_t channel,
				uint16_t min_value,
				uint16_t max_value,
				uint8_t measurement_error);
		PWMCapturer(TIM_HandleTypeDef *htim,
				uint8_t channel);
		virtual ~PWMCapturer();
		void calculatePulseWidth();
		uint32_t getPulseWidthDif();
		uint32_t getPulseWidth();
		uint32_t getMinWidth();
		uint32_t getMidWidth();
		uint32_t getMaxWidth();
		bool matchMinValue();
		bool matchMidValue();
		bool matchMaxValue();
		bool matchOutOfInterval();
		bool matchValue(uint16_t value);

	private:
		uint16_t max_value = 0;
		uint16_t mid_value = 0;
		uint16_t min_value = 0;
		uint32_t IC_Val1 = 0;
		uint32_t IC_Val2 = 0;
		uint32_t difference = 0;
		bool is_First_Captured = false;
		uint32_t channel;
		TIM_HandleTypeDef *htim;
		uint8_t measurement_error;
};

#endif /* PWMCAPTURER_PWMCAPTURER_H_ */
