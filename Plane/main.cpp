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

class RcChannels
{
private:
    //static EventFlags rc_channels; // очередь флагов для всех каналов(объектов) одна (вроде и не нужна когда есть очередь событий)
    //uint32_t PIN_UP_FLAG;
    //uint32_t PIN_DOWN_FLAG;
    bool const start{1};
    bool const stop{0};
    
    Timer timer;
    int pulse_width;
    void calcPulseWidth(bool const);
    Event<void()> start_timer  = queue.event(this, &RcChannels::calcPulseWidth, start);
    Event<void()> stop_timer  = queue.event(this, &RcChannels::calcPulseWidth, stop);
    
public:
    void upHandler();
    void downHandler();
    int getPulseWidth();
    RcChannels(InterruptIn);
    ~RcChannels();
};

void RcChannels::upHandler()
{
    //rc_channels.set(this->PIN_UP_FLAG);
    //тут будет функция которая ставит в очередь старт таймера для этого канала
    //Event<void(bool const)>    e  = queue.event(this, &Channels::calcPulseWidth);
    start_timer.post();
}
void RcChannels::downHandler()
{
    //rc_channels.set(this->PIN_DOWN_FLAG);
    //тут будет функция которая ставит в очередь стоп таймера для этого канала
    stop_timer.post();
}
void RcChannels::calcPulseWidth(bool start_or_stop_timer)
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
int RcChannels::getPulseWidth()
{
    return pulse_width;
}
RcChannels::RcChannels(InterruptIn pin)
{
    //добавить присвоение портов и обработчиков прерывваний
    pin.mode(PullDown);
    pin.rise(this, &RcChannels::upHandler);
    pin.fall(this, &RcChannels::downHandler);
}
RcChannels::~RcChannels()
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

    radio_control_th.start(callback(&queue, &EventQueue::dispatch_forever));
    //radio_control_th.start(setLedOutPwm);
}