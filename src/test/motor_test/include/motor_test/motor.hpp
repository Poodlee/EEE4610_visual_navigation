// pwm.hpp
#ifndef pwm_hpp
#define pwm_hpp

#include <wiringPi.h>
#include <chrono>
#include <thread>
#include <math.h>

double angle;
std::chrono::system_clock::time_point fall_start, fall_end;
bool isFreeFall;
bool freefalled = false;
bool start_set = false;

class PWM{
  public:
    PWM(unsigned short pin);
    ~PWM();

    void testing(float duty = 0.272f);
    void pulse(float freq, float time, float duty = 0.15f);
    void pulses(float freq, unsigned long pulses, float duty = 0.15f);

  private:
    unsigned short pin;

};

#endif
