// pwm.cpp
#include "motor_test/motor.hpp"

#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>

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
  isFreeFall = false;
  double net_accel =
      std::sqrt((msg->linear_acceleration.x * msg->linear_acceleration.x) +
                (msg->linear_acceleration.y * msg->linear_acceleration.y) +
                (msg->linear_acceleration.z * msg->linear_acceleration.z));

  angle = asin(msg->linear_acceleration.x / 9.8) / 3.141592 * 180;
  stringstream ss;
  ss << angle;
  
  ROS_INFO("angle: %s", ss.str().c_str());
  
  PWM test(16); // 16- > GPIO 16

  if (net_accel < 5.0) {
    if (start_set == false) {
      fall_start = std::chrono::system_clock::now();
      start_set = true;
    }
    isFreeFall = true;
    

    if (angle <= 90 && angle > 5) {
      test.testing(0.372);
      test.testing(0.372);
    }
    freefalled = true;
  }

  if (isFreeFall == false && freefalled == true) {
    if (angle >= -5 && angle <= 5) {
      fall_end = std::chrono::system_clock::now();
      std::chrono::milliseconds milli = std::chrono::duration_cast<std::chrono::milliseconds> (fall_end - fall_start);
      string s = to_string((int)milli.count());
      
      ofstream fout;
      fout.open("/home/ubuntu/result.txt", ios::out | ios::app);
      if (!fout) {
        ROS_INFO("Open Failed");
      }
      fout << s << "\n";
      ROS_INFO("Save completed");
      fout.close();
      freefalled = false;
      start_set = false;
      test.testing(0.172);
    }
  }
}

int main(int argc, char **argv){
  wiringPiSetupGpio(); // Initalize Pi GPIO
  // float trans;

  ros::init(argc, argv, "motor_control");
  ros::NodeHandle nh;

  ros::Subscriber sub = nh.subscribe("/imu/data",100,msgCallback);

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
