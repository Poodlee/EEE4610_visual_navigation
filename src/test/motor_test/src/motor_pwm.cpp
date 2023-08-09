#include "motor_pwm.hpp"

#include <unistd.h>

#include <chrono>
#include <fstream>
#include <math.h>
#include <string>
#include <sstream>

PWM::PWM(unsigned short pin) : pin(pin){
  pinMode(pin, OUTPUT);
}

PWM::~PWM(){
  digitalWrite(pin, LOW);
}

// freq = 182.25 -> 5486.9684 (10^6 / freq) && duty = 0.172(Left) ~ 0.372(Right)
void PWM::testing(float duty){
    unsigned int delayHigh = round((5486.9684 * duty)) - 88;
    unsigned int delayLow = round(5486.9684 * (1.0f - duty)) - 88;
    digitalWrite(pin,HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(delayHigh));
    digitalWrite(pin, LOW);
    std::this_thread::sleep_for(std::chrono::microseconds(delayLow));
}

void PWM::pulse(float freq, float time, float duty){
  std::chrono::steady_clock::time_point endPoint = std::chrono::steady_clock::now() + std::chrono::milliseconds(int(round(time)));

  unsigned int delayHigh = round((1000000.0f / freq) * duty) - 76;
  unsigned int delayLow = round((1000000.0f / freq) * (1.0f - duty)) - 76;

  while(endPoint > std::chrono::steady_clock::now()){
    digitalWrite(pin, HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(delayHigh));
    digitalWrite(pin, LOW);
    std::this_thread::sleep_for(std::chrono::microseconds(delayLow));
  }
}

void PWM::pulses(float freq, unsigned long pulses, float duty){
  unsigned int delayHigh = round((1000000.0f / freq) * duty) - 76;
  unsigned int delayLow = round((1000000.0f / freq) * (1.0f - duty)) - 76;

  for(unsigned long i = 0; i < pulses; i++){
    digitalWrite(pin, HIGH);
    std::this_thread::sleep_for(std::chrono::microseconds(delayHigh));
    digitalWrite(pin, LOW);
    std::this_thread::sleep_for(std::chrono::microseconds(delayLow));
  }
}
