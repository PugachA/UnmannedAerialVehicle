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
PinName elevator_pin = PA_1;

bool flag_print{0};
int global_pulse_throttle{0};
int global_pulse_elevator{0};
 
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
    RcChannels throttle(throttle_pin), elevator(elevator_pin);
    
    while(1)
    {
        throttle.savePulseWidth();
        elevator.savePulseWidth();
        global_pulse_throttle = throttle.getPulseWidth();
        global_pulse_elevator = elevator.getPulseWidth();
    }
}
int main()
{
    Serial pc(USBTX, USBRX);

    Ticker printer;
    printer.attach(printPulseWidth, 1.0);
    
    radio_control_th.start(secondMain);

    while(1)
    {
        if(flag_print)
        {
            flag_print = 0;
            printf("%d,  %d\n", global_pulse_throttle, global_pulse_elevator);
        }
    }
}