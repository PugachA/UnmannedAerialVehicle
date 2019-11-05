#include "mbed.h"
#include "rtos.h"
#include "Math/Math.h"

PwmOut red_led(LED1);
InterruptIn button_up(PA_3);
InterruptIn button_down(PA_5);

uint32_t const BUTTON_DOWN = 1 << 0;
uint32_t const BUTTON_UP = 1 << 1;

Thread led_control_th;

EventFlags buttons;

/*float limiter(float max, float min, float value)
{
    return value >= max ? max : value <= min ? min : value;
}*/
void buttonUpHandler()
{
    buttons.set(BUTTON_UP);
}
void buttonDownHandler()
{
    buttons.set(BUTTON_DOWN);
}
void setLedOutPwm()
{  
    float duty_cycle{0};
    uint32_t button_read_flag{0};

    while(1){
        button_read_flag = buttons.wait_any(BUTTON_UP | BUTTON_DOWN);
        switch (button_read_flag)
        {
            case BUTTON_UP: red_led.pulsewidth_us(duty_cycle = Math::limiter<float>(1875, 900, duty_cycle + 100)); break;
            case BUTTON_DOWN: red_led.pulsewidth_us(duty_cycle = Math::limiter<float>(1875, 900, duty_cycle - 100)); break;  
            
            default: break;
        }
        ThisThread::sleep_for(5);
    }
}

int main()
{
    red_led = 0;

    button_up.mode(PullDown);
    button_down.mode(PullDown);

    button_down.rise(&buttonDownHandler);
    button_up.rise(&buttonUpHandler);
    
    led_control_th.start(setLedOutPwm);
}