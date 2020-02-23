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
    bool stop_timer{0};
    InterruptIn pin;
    Timer timer;
    int pulse_width;
    //Event<void()> start_timer  = queue.event(this, &RcChannels::calcPulseWidth, start);
    
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
    timer.start();
}
void RcChannels::downHandler()
{
    timer.stop();
    stop_timer = 1;
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
}