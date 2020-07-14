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

PinName throttle_servo_pin = PA_3;
// PinName yaw = PA_0;

PinName throttle_pin = PE_12;
// PinName elevator_pin = PA_1;
// PinName aileron_pin = PB_7;
// PinName gear_pin = PD_0;
// PinName rudder_pin = PB_6;

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
    // pin.mode(PullDown);
    pin.rise(callback(this, &RcChannels::upHandler));
    pin.fall(callback(this, &RcChannels::downHandler));
}
RcChannels::~RcChannels()
{
}

class Servo
{
private:
    const float DEFAULT_SERVO_PWM_PERIOD{0.02}; //50 Гц
    const float DEFAULT_SERVO_PULSE_WIDTH{1500}; //1500 мкС
    float servo_signal{0};
    PwmOut pin;

    //Event<void()> update_servo_pos = queue.event(this, Servo::setPositionUs, );
public:
    void setPositionUs(int duty_sycle_us);
    Servo(PinName);
    ~Servo();
};
void Servo::setPositionUs(int pulse_width)
{
    pin.pulsewidth_us(pulse_width);
}
Servo::Servo(PinName pin_name) : pin(pin_name)
{
    pin.period(DEFAULT_SERVO_PWM_PERIOD);
    pin.pulsewidth_us(DEFAULT_SERVO_PULSE_WIDTH);
}
Servo::~Servo()
{
}

class DirectMode
{
private:
    RcChannels signal_in[5];
    Servo signal_out[5];

public:
    void updateSignalIn(RcChannels*);
    void updateSignalOut(Servo*);
    void updateMode();
};

void DirectMode::updateSignalIn(RcChannels* input)
{
 
}

void DirectMode::updateSignalOut(Servo* output)
{

}

void DirectMode::updateMode()
{

}

void printPulseWidth()
{
    flag_print = true;
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
    // RcChannels throttle(throttle_pin), elevator(elevator_pin), rudder(rudder_pin), aileron(aileron_pin), gear(gear_pin);
    RcChannels throttle(throttle_pin);
    Servo throttle_servo(throttle_servo_pin);
    Ticker printer;
    printer.attach(printPulseWidth, 0.01);
    
    radio_control_th.start(callback(&queue, &EventQueue::dispatch_forever));

    while(1)
    {
        throttle_servo.setPositionUs(throttle.getPulseWidth());
        if(flag_print)
        {
            flag_print = false;
            printf("%d\n", throttle.getPulseWidth());//, rudder.getPulseWidth(), elevator.getPulseWidth(), aileron.getPulseWidth(), gear.getPulseWidth());
        }
    }
}