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
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	HAL_Delay(SHORT_BEEP_TIME_MS);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
void Beeper::longBeep()
{
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_SET);
	HAL_Delay(LONG_BEEP_TIME_MS);
	HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
}
void Beeper::seriesBeep()
{
	shortBeep();
	HAL_Delay(SHORT_BEEP_TIME_MS);
	shortBeep();
	HAL_Delay(SHORT_BEEP_TIME_MS);
	shortBeep();
}