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
		uint32_t multiplier; // кол-во тактов за 1 мкс
	
	public:
		SimpleClock();
		void Delay(uint32_t micro_seconds);
		void Restart();
		uint32_t GetTime();
};
