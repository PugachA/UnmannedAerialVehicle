#include "Beeper.h"

Beeper::Beeper(GPIO_TypeDef * port, uint32_t pin)
{
	this->port = port;
	this->pin = pin;
}

Beeper::~Beeper()
{
}

void Beeper::shortBeep()
{
	this->beep(SHORT_BEEP_TIME_MS);
}

void Beeper::longBeep()
{
	this->beep(LONG_BEEP_TIME_MS);
}

void Beeper::seriesBeep()
{
	shortBeep();
	HAL_Delay(SHORT_BEEP_TIME_MS);
	shortBeep();
	HAL_Delay(SHORT_BEEP_TIME_MS);
	shortBeep();
}

void Beeper::beep(uint16_t milli_Seconds)
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	HAL_Delay(milli_Seconds);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}

//Использовать надо аккуратно. Не поддерживает множественный вызов и много поточность.
//Для одного объекта должен быть один вызов.
void Beeper::seriesBeepAsync(uint16_t period_Milli_Seconds)
{
	if(this->stop_Time_Stamp == 0)
	{
		this->stop_Time_Stamp = HAL_GetTick() + period_Milli_Seconds;
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	}

	if(this->stop_Time_Stamp <= HAL_GetTick())
		HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);

	if(this->stop_Time_Stamp + period_Milli_Seconds <= HAL_GetTick())
		this->stop_Time_Stamp = 0;
}
