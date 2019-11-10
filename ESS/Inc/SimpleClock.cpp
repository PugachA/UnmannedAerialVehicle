#include "SimpleClock.h"

#define DWT_CYCCNT    *(volatile unsigned long *)0xE0001004
#define DWT_CONTROL   *(volatile unsigned long *)0xE0001000
#define SCB_DEMCR     *(volatile unsigned long *)0xE000EDFC
	
SimpleClock::SimpleClock()
{
	SCB_DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // разрешаем использовать счётчик
	DWT_CONTROL |= DWT_CTRL_CYCCNTENA_Msk;   // запускаем счётчик
}

void SimpleClock::Delay(uint32_t microSeconds)
{
	uint32_t count_tic =  microSeconds * (SystemCoreClock / 1000000); // получаем кол-во тактов за 1 мкс и умножаем на наше значение
	DWT->CYCCNT = 0; // обнуляем счётчик
	while(DWT->CYCCNT < count_tic);
}