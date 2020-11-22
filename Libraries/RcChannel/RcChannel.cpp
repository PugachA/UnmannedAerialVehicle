#include "RcChannel.h"

//external global variables:
//----------------------
extern TIM_HandleTypeDef htim2, htim5;
//----------------------

RcChannel::RcChannel(TIM_HandleTypeDef *htim, uint8_t channel_num, uint16_t channel_min_value, uint16_t channel_max_value)
{
	this->channel_min_value = channel_min_value;
	this->channel_max_value = channel_max_value;
	this->channel_mid_value = channel_min_value + (channel_max_value - channel_min_value)/2;

	this->htim = htim;
	switch(channel_num)
	{
		case 1: tim_channel = TIM_CHANNEL_1; break;
		case 2: tim_channel = TIM_CHANNEL_2; break;
		case 3: tim_channel = TIM_CHANNEL_3; break;
		case 4: tim_channel = TIM_CHANNEL_4; break;
	}
}
RcChannel::RcChannel(TIM_HandleTypeDef *htim, uint8_t channel_num, uint16_t channel_min_value, uint16_t channel_mid_value, uint16_t channel_max_value)
{
	this->channel_min_value = channel_min_value;
	this->channel_max_value = channel_max_value;
	this->channel_mid_value = channel_mid_value;

	this->htim = htim;
	switch(channel_num)
	{
		case 1: tim_channel = TIM_CHANNEL_1; break;
		case 2: tim_channel = TIM_CHANNEL_2; break;
		case 3: tim_channel = TIM_CHANNEL_3; break;
		case 4: tim_channel = TIM_CHANNEL_4; break;
	}
}
RcChannel::~RcChannel()
{

}
uint32_t RcChannel::getPulseWidth()
{
	return Difference;
}
uint32_t RcChannel::getChannelMaxWidth()
{
	return channel_max_value;
}
uint32_t RcChannel::getChannelMinWidth()
{
	return channel_min_value;
}
uint32_t RcChannel::getChannelMidWidth()
{
	return channel_mid_value;
}
void RcChannel::pulseWidthCalc()
{
	if (Is_First_Captured==0) // if the first value is not captured
	{
		IC_Val1 = HAL_TIM_ReadCapturedValue(this->htim, tim_channel); // read the first value
		Is_First_Captured = 1;  // set the first captured as true
		// Now change the polarity to falling edge
		__HAL_TIM_SET_CAPTUREPOLARITY(this->htim, tim_channel, TIM_INPUTCHANNELPOLARITY_FALLING);
	}
	else
		if (Is_First_Captured==1)   // if the first is already captured
		{
			IC_Val2 = HAL_TIM_ReadCapturedValue(this->htim, tim_channel);  // read second value
			//__HAL_TIM_SET_COUNTER(htim, 0);  // reset the counter

			if (IC_Val2 > IC_Val1)
			{
				Difference = IC_Val2-IC_Val1;
			}
			Is_First_Captured = 0; // set it back to false
			// set polarity to rising edge
			__HAL_TIM_SET_CAPTUREPOLARITY(this->htim, tim_channel, TIM_INPUTCHANNELPOLARITY_RISING);
		}
}
uint32_t RcChannel::getPulseWidthDif()
{
	return  ((int16_t)(this->channel_mid_value) - ((int16_t)(this->Difference) - (int16_t)(this->channel_mid_value)));
}
uint8_t RcChannel::matchMinValue()
{
	if( (this->Difference > (this->channel_min_value - 30)) && (this->Difference < (this->channel_min_value + 30)) )
	{
		return 1;
	} else
	{
		return 0;
	}
}
uint8_t RcChannel::matchMidValue()
{
	if( (this->Difference > (this->channel_mid_value - 30)) && (this->Difference < (this->channel_mid_value + 30)) )
	{
		return 1;
	} else
	{
		return 0;
	}
}
uint8_t RcChannel::matchMaxValue()
{
	if( (this->Difference > (this->channel_max_value - 30)) && (this->Difference < (this->channel_max_value + 30)) )
	{
		return 1;
	} else
	{
		return 0;
	}
}
RcChannel thr_rc(&htim2, 1, 989, 1500, 2013), elev_rc(&htim2, 2, 989, 1500, 2013),
		  ail_rc(&htim2, 3, 989, 1500, 2013), rud_rc(&htim2, 4, 989, 1500, 2015),
		  switch_rc(&htim5, 1, 989, 1500, 2013), slider_rc(&htim5, 2, 989, 2013);
		  
void IcHandlerTim2(TIM_HandleTypeDef *htim)
{
	switch ( (uint8_t) htim->Channel )
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
		{
			thr_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
		{
			elev_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
		{
			ail_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
		{
			rud_rc.pulseWidthCalc();
		} break;
	}
}

void IcHandlerTim5(TIM_HandleTypeDef *htim)
{
	switch ( (uint8_t) htim->Channel )
	{
		case HAL_TIM_ACTIVE_CHANNEL_1:
		{
			switch_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_2:
		{
			slider_rc.pulseWidthCalc();
		} break;
		case HAL_TIM_ACTIVE_CHANNEL_3:
		{

		} break;
		case HAL_TIM_ACTIVE_CHANNEL_4:
		{

		} break;
	}
}
