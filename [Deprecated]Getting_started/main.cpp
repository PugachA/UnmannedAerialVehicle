#include "Sum\sum.h"
#include "mbed.h"

DigitalOut led1(LED1);

int main()
{
	int result{0};
    while(true)
    {
        result = sum(10, 12);
        led1 = !led1;
        wait(2);
    }
}