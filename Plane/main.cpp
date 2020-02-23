#include "mbed.h"
#include "rtos.h"
#include "Math/Math.h"

//DigitalOut green_led(LED1);
DigitalOut red_led(LED2);

/*PinName throttle_pin = PA_3;
//InterruptIn throttle_pin(PA_3);
InterruptIn elevator_pin(PA_5);
InterruptIn aileron_pin(PA_5);*/

Thread radio_control_th;
EventQueue queue;
PinName throttle_pin = PA_2;
bool flag_print{0};
Mail<int, 16> mail_box;

class RcChannels
{
private:
    //static EventFlags rc_channels; // очередь флагов для всех каналов(объектов) одна (вроде и не нужна когда есть очередь событий)
    //uint32_t PIN_UP_FLAG;
    //uint32_t PIN_DOWN_FLAG;
    //bool const start{1};
    //bool const stop{0};
    bool stop_timer{0};
    InterruptIn pin;
    Timer timer;
    int pulse_width;
    //Event<void()> start_timer  = queue.event(this, &RcChannels::calcPulseWidth, start);
    Event<void()> save_pulse_width  = queue.event(this, &RcChannels::savePulseWidth);
    
    void upHandler();
    void downHandler();
    
public:
    int getPulseWidth();
    void savePulseWidth();
    RcChannels(PinName);
    ~RcChannels();
};

void RcChannels::upHandler()
{
    //rc_channels.set(this->PIN_UP_FLAG);
    //тут будет функция которая ставит в очередь старт таймера для этого канала
    //Event<void(bool const)>    e  = queue.event(this, &Channels::calcPulseWidth);
    //start_timer.post();
    //red_led = 1;
    timer.start();
    //start_or_stop_timer = 1;
}
void RcChannels::downHandler()
{
    //rc_channels.set(this->PIN_DOWN_FLAG);
    //тут будет функция которая ставит в очередь стоп таймера для этого канала
    //stop_timer.post();
    timer.stop();
    stop_timer = 1;
    //save_pulse_width.post();
    //pulse_width = timer.read_us();    
    //timer.reset();
    //red_led = 0;
    //start_or_stop_timer = 0;
}
void RcChannels::savePulseWidth()
{
    if(stop_timer)
    {
        pulse_width = timer.read_us();
        timer.reset();
        stop_timer = 0;
        ThisThread::sleep_for(10);
    }
}
int RcChannels::getPulseWidth()
{
    return pulse_width;
}
RcChannels::RcChannels(PinName pin_name) : pin(pin_name)
{
    //добавить присвоение портов и обработчиков прерывваний
    pin.mode(PullDown);
    pin.rise(callback(this, &RcChannels::upHandler));
    pin.fall(callback(this, &RcChannels::downHandler));
}
RcChannels::~RcChannels()
{
}

void printPulseWidth()
{
    flag_print = 1;
}
void secondMain()
{
    RcChannels throttle(throttle_pin);
    int* mail = mail_box.alloc();
     
    while(1)
    {
        throttle.savePulseWidth();
        *mail = throttle.getPulseWidth();
        mail_box.put(mail);
    }
}
int main()
{
    Serial pc(USBTX, USBRX);

    Ticker printer;
    printer.attach(printPulseWidth, 1.0);
    
    radio_control_th.start(secondMain);

    int* mail;
    osEvent evt;

    while(1)
    {
        evt = mail_box.get();
        mail = (int*) evt.value.p;
        if(flag_print)
        {
            flag_print = 0;
            if(evt.status == osEventMail)
                printf("%d\n", *mail);
        }
        mail_box.free(mail);

    }
    //ВОТ ЭТОТ КУСОК РАБОТАЕТ НЕ ТРОГАЙ

    /*Serial pc(USBTX, USBRX);
    red_led = 0;

    EventQueue queue;
    Timer t;
    int timer_value{0};

    Event<void(Timer*)> start_timer  = queue.event(&upHandler);
    Event<void(Timer*)> stop_timer  = queue.event(&downHandler);
    Event<void(DigitalOut)> blink  = queue.event(&ledBlink);

    radio_control_th.start(callback(&queue, &EventQueue::dispatch_forever));
    ThisThread::sleep_for(1000);
    blink.post(red_led);
    while(1)
    {
        //red_led = !red_led;
        start_timer.post(&t);
        ThisThread::sleep_for(1000);
        timer_value = t.read_ms();
        printf("%d\n", timer_value);
        stop_timer.post(&t);
        t.reset();
    }*/
}