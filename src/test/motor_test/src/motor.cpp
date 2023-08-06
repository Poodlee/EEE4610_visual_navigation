// pwm.cpp
#include "motor_test/motor.hpp"

#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"  // Add Topic msg header

using namespace std;

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
    this_thread::sleep_for(chrono::microseconds(delayHigh));
    digitalWrite(pin, LOW);
    this_thread::sleep_for(chrono::microseconds(delayLow));
}

void PWM::pulse(float freq, float time, float duty){
  chrono::steady_clock::time_point endPoint = chrono::steady_clock::now() + chrono::milliseconds(int(round(time)));

  unsigned int delayHigh = round((1000000.0f / freq) * duty) - 76;
  unsigned int delayLow = round((1000000.0f / freq) * (1.0f - duty)) - 76;

  while(endPoint > chrono::steady_clock::now()){
    digitalWrite(pin, HIGH);
    this_thread::sleep_for(chrono::microseconds(delayHigh));
    digitalWrite(pin, LOW);
    this_thread::sleep_for(chrono::microseconds(delayLow));
  }
}

void PWM::pulses(float freq, unsigned long pulses, float duty){
  unsigned int delayHigh = round((1000000.0f / freq) * duty) - 76;
  unsigned int delayLow = round((1000000.0f / freq) * (1.0f - duty)) - 76;

  for(unsigned long i = 0; i < pulses; i++){
    digitalWrite(pin, HIGH);
    this_thread::sleep_for(chrono::microseconds(delayHigh));
    digitalWrite(pin, LOW);
    this_thread::sleep_for(chrono::microseconds(delayLow));
  }
}

void msgCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
  ROS_INFO("Received value from imu");
  angle = msg->orientation.x;
}

int main(int argc, char **argv){
  wiringPiSetupGpio(); // Initalize Pi GPIO
  PWM test(16); // 16- > GPIO 16
  float trans;
  std::chrono::system_clock::time_point start, end;

  ros::init(argc, argv, "motor_control");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/imu/data",100,msgCallback);
  if (angle <= 90){
    start = std::chrono::system_clock::now();
    test.testing(0.372);
  }
  if (angle == 0){
    end = std::chrono::system_clock::now();
    std::chrono::milliseconds milli = std::chrono::duration_cast<std::chrono::milliseconds> (end - start);
    string s = to_string(milli.count());
    ofstream fout;
    fout.open("result.txt", ios::out | ios::app);
    fout << s << "\n";
    ROS_INFO("Save completed");
  }

  ros::spin();

  return 0;
  // while(1){
  //   test.testing(0.188);
    // for(int i=172;i<=372; i+=50){
    //   trans = (float) i / 1000;
    //   cout << "First: " << trans << endl;
    //   test.testing(trans);
    //   sleep(2);
    // }
    // for(int i=372;i>=172; i-=50){
    //   trans = (float) i / 1000;
    //   cout << "Second: " << trans << endl;
    //   test.testing(trans);
    //   sleep(2);
    // }
  //   cout << "---pulse transmitting---" << endl;

  // }
}
