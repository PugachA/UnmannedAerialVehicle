/**
  ******************************************************************************
  * @file           : SimpleClock.h
  * @brief          : Класс для работы со временем через DWT (Data Watchpoint and Trace Unit)
  ******************************************************************************
  */

#include "stm32f3xx_hal.h"

class SimpleClock
{
	private:
		uint32_t ticCounter;
		uint32_t microSecondCounter;
	
	public:
		SimpleClock();
		void Delay(uint32_t microSeconds);
};
