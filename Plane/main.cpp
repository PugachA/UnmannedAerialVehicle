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
PinName aileron_pin = PB_7;
PinName gear_pin = PD_0;
PinName rudder_pin = PB_6;

bool flag_print{0};
int global_pulse_throttle{0};
int global_pulse_elevator{0};
 
class RcChannels
{
private:
    
    InterruptIn pin;
    Timer timer;
    volatile int pulse_width;
    Event<void()> save_pulse_width  = queue.event(this, &RcChannels::savePulseWidth);
    
    void savePulseWidth();
    void upHandler();
    void downHandler();
    
public:
    int getPulseWidth();
    
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
    queue.call(save_pulse_width);
}
void RcChannels::savePulseWidth()
{
    pulse_width = timer.read_us();
    timer.reset();
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
/*void receiver_th()
{
    RcChannels throttle(throttle_pin), elevator(elevator_pin);

    while(1)
    {
        global_pulse_throttle = throttle.getPulseWidth();
        global_pulse_elevator = elevator.getPulseWidth();
    }
}*/
int main()
{
    RcChannels throttle(throttle_pin), elevator(elevator_pin), rudder(rudder_pin), aileron(aileron_pin), gear(gear_pin);
    Serial pc(USBTX, USBRX);

    Ticker printer;
    printer.attach(printPulseWidth, 0.01);
    
    radio_control_th.start(callback(&queue, &EventQueue::dispatch_forever));

    while(1)
    {
        if(flag_print)
        {
            flag_print = 0;
            printf("%d,  %d,  %d,  %d,  %d\n", throttle.getPulseWidth(), rudder.getPulseWidth(), elevator.getPulseWidth(), aileron.getPulseWidth(), gear.getPulseWidth());
        }
    }
}