#include "SimpleClock.h"

#define DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC
	
SimpleClock::SimpleClock()
{
	this->multiplier = SystemCoreClock / 1000000; // получаем кол-во тактов за 1 мкс
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

void SimpleClock::Delay(uint32_t micro_seconds)
{
	// получаем кол-во тактов, которое нужно выполнить, чтобы достичь зад. времени
	uint32_t count_tic = this->GetTime() + micro_seconds * (SystemCoreClock / 1000000);
	
	while((int32_t)(this->GetTime() - count_tic) < 0);
}

void SimpleClock::Restart()
{
	DWT->CYCCNT = 0; //обнуляем счетчик
}

uint32_t SimpleClock::GetTime()
{
	this->multiplier = SystemCoreClock / 1000000; //ебанная магия, если закомментить эту строку значение микросекунд будет в 10 раз меньше
	uint32_t micro_seconds = DWT->CYCCNT / this->multiplier;
	return micro_seconds; //получаем пройженное количество микросекунд
}