#include "mbed.h"
#include "rtos.h"
#include "Math/Math.h"

PwmOut red_led(LED1);
InterruptIn throttle_pin(PA_3);
InterruptIn elevator_pin(PA_5);
InterruptIn aileron_pin(PA_5);

/*EventFlags rc_channels;
EventFlags rc_channels_down;
uint32_t const THROTTEL_PIN_UP = 1 << 0;
uint32_t const THROTTEL_PIN_DOWN = 1 << 1;
uint32_t const ELEVATOR_PIN_UP = 1 << 2;
uint32_t const ELEVATOR_PIN_DOWN = 1 << 3;
uint32_t const AILERON_PIN_UP = 1 << 4;
uint32_t const AILERON_PIN_DOWN = 1 << 5;*/


Thread radio_control_th;
EventQueue queue(32 * EVENTS_EVENT_SIZE);

class Channels
{
private:
    static EventFlags rc_channels; // очередь флагов для всех каналов(объектов) одна

    uint32_t PIN_UP_FLAG;
    uint32_t PIN_DOWN_FLAG;
    Timer timer;
    int pulse_width;
    int calcPulseWidth(bool);
public:
    void upHandler();
    void downHandler();
    int getPulseWidth();
    Channels(/* args */);
    ~Channels();
};

void Channels::upHandler()
{
    rc_channels.set(this->PIN_UP_FLAG);
}
void Channels::downHandler()
{
    rc_channels.set(this->PIN_DOWN_FLAG);
}
int Channels::calcPulseWidth(bool start_or_stop_timer)
{
    if(start_or_stop_timer)
        
    timer.stop();
}
int Channels::getPulseWidth()
{

}
Channels::Channels(/* args */)
{
    //тут механизм соответствия флагов в очереди состояниям пинов
    this -> PIN_UP_FLAG = 1 << 0;//времянка пока не придумаю механизм
    this -> PIN_DOWN_FLAG = 1 << 2;//времянка пока не придумаю механизм
}

Channels::~Channels()
{
}


/*void throttleUpHandler()
{
    rc_channels.set(THROTTEL_PIN_UP);
}
void throttleDownHandler()
{
    rc_channels.clear(THROTTEL_PIN_DOWN);
}*/
int pulseWidthCalc(Timer t, bool start_or_stop)
{
    if(start_or_stop)
        t.start();
    else
    {
        t.stop();
        int result = t.read();
        t.reset();
        return result;
    }
    
    
}
void readRC()
{
    uint32_t current_channel{0};
    while(1)
    {
        current_channel = rc_channels.wait_any(THROTTEL_PIN_UP | THROTTEL_PIN_DOWN | ELEVATOR_PIN_DOWN | ELEVATOR_PIN_UP | AILERON_PIN_DOWN | AILERON_PIN_UP);
        
    
    }
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

    throttle_pin.mode(PullDown);
    elevator_pin.mode(PullDown);
    aileron_pin.mode(PullDown);

    throttle_pin.rise(&throttleUpHandler);
    throttle_pin.rise(&throttleDownHandler);
    
    radio_control_th.start(setLedOutPwm);
}