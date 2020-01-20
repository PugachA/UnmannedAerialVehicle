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
EventQueue queue;

class Channels
{
private:
    //static EventFlags rc_channels; // очередь флагов для всех каналов(объектов) одна (вроде и не нужна когда есть очередь событий)
    bool const start{1};
    bool const stop{0};    

    uint32_t PIN_UP_FLAG;
    uint32_t PIN_DOWN_FLAG;
    Timer timer;
    int pulse_width;
    void calcPulseWidth(bool const);
    Event<void()> start_timer  = queue.event(this, &Channels::calcPulseWidth, start);
    Event<void()> stop_timer  = queue.event(this, &Channels::calcPulseWidth, stop);
    
public:
    void upHandler();
    void downHandler();
    int getPulseWidth();
    Channels(/* args */);
    ~Channels();
};

void Channels::upHandler()
{
    //rc_channels.set(this->PIN_UP_FLAG);
    //тут будет функция которая ставит в очередь старт таймера для этого канала
    //Event<void(bool const)>    e  = queue.event(this, &Channels::calcPulseWidth);
    start_timer.post();
}
void Channels::downHandler()
{
    //rc_channels.set(this->PIN_DOWN_FLAG);
    //тут будет функция которая ставит в очередь стоп таймера для этого канала
}
void Channels::calcPulseWidth(bool start_or_stop_timer)
{
    if(start_or_stop_timer)
        timer.start();
    else
    {
        pulse_width = timer.read_us();
        timer.stop();
        timer.reset();
    }   
}
int Channels::getPulseWidth()
{
    return pulse_width;
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

int main()
{
    //red_led = 0;

    throttle_pin.mode(PullDown);
    elevator_pin.mode(PullDown);
    //aileron_pin.mode(PullDown);

    //throttle_pin.rise(&throttleUpHandler);
    //throttle_pin.rise(&throttleDownHandler);
    radio_control_th.start(callback(&queue, &EventQueue::dispatch_forever));
    //radio_control_th.start(setLedOutPwm);
}