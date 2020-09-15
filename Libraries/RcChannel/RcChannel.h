#ifndef RCCHANNEL_H
#define RCCHANNEL_H

#include "stm32f4xx_hal.h"

class RcChannel
{
	private:
		uint16_t channel_max_value = 0;
		uint16_t channel_mid_value = 0;
		uint16_t channel_min_value = 0;
		uint32_t IC_Val1 = 0;
		uint32_t IC_Val2 = 0;
		uint32_t Difference = 0;
		uint8_t Is_First_Captured = 0;
		uint32_t tim_channel = 0;
		TIM_HandleTypeDef *htim;
	public:
		void pulseWidthCalc();
		uint32_t getPulseWidthDif();
		uint32_t getPulseWidth();
		uint32_t getChannelMaxWidth();
		uint32_t getChannelMinWidth();
		uint32_t getChannelMidWidth();
		uint8_t matchMinValue();
		uint8_t matchMidValue();
		uint8_t matchMaxValue();
		
		RcChannel(TIM_HandleTypeDef *htim, uint8_t channel_num, uint16_t channel_min_value, uint16_t channel_max_value);
		RcChannel(TIM_HandleTypeDef *htim, uint8_t channel_num, uint16_t channel_min_value, uint16_t channel_mid_value, uint16_t channel_max_value);
		~RcChannel();

};

void IcHandlerTim2(TIM_HandleTypeDef *htim);
void IcHandlerTim5(TIM_HandleTypeDef *htim);

#endif // RCCHANNEL_H
