#ifndef BEEPER_H
#define BEEPER_H

#include "stm32f4xx_hal.h"

class Beeper
{
	private:
		GPIO_TypeDef * port = 0;
		uint32_t pin = 0;
		const uint16_t LONG_BEEP_TIME_MS = 1000;
		const uint16_t SHORT_BEEP_TIME_MS = 100;
		uint32_t stop_Time_Stamp = 0;

	public:
		Beeper(GPIO_TypeDef * port, uint32_t pin);
		~Beeper();
		void shortBeep();
		void longBeep();
		void seriesBeep();
		void beep(uint16_t milli_Seconds);
		void seriesBeepAsync(uint16_t period_Milli_Seconds);
};

#endif // BEEPER_H
