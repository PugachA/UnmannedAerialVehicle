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
		uint32_t stopTimeStamp = 0;

	public:
		Beeper(GPIO_TypeDef * port, uint32_t pin);
		~Beeper();
		void shortBeep();
		void longBeep();
		void seriesBeep();
		void beep(uint16_t milliSeconds);
		void seriesBeepAsync(uint16_t milliSeconds);
};

#endif // BEEPER_H
