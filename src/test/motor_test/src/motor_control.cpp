#include "motor_pwm.hpp"
#include "imu_uart.hpp"
#include <iostream>

// #include "ros/ros.h"
// #include "sensor_msgs/Imu.h"  // Add Topic msg header


// void msgCallback(const sensor_msgs::Imu::ConstPtr& msg)
// {
//   isFreeFall = false;
//   double net_accel =
//       std::sqrt((msg->linear_acceleration.x * msg->linear_acceleration.x) +
//                 (msg->linear_acceleration.y * msg->linear_acceleration.y) +
//                 (msg->linear_acceleration.z * msg->linear_acceleration.z));

//   angle = asin(msg->linear_acceleration.x / 9.8) / 3.141592 * 180;
//   stringstream ss;
//   ss << angle;

//   ROS_INFO("angle: %s", ss.str().c_str());

//   PWM test(16); // 16- > GPIO 16

//   if (net_accel < 5.0) {
//     if (start_set == false) {
//       fall_start = std::chrono::system_clock::now();
//       start_set = true;
//     }
//     isFreeFall = true;


//     if (angle <= 90 && angle > 5) {
//       test.testing(0.372);
//       test.testing(0.372);
//     }
//     freefalled = true;
//   }

//   if (isFreeFall == false && freefalled == true) {
//     if (angle >= -5 && angle <= 5) {
//       fall_end = std::chrono::system_clock::now();
//       std::chrono::milliseconds milli = std::chrono::duration_cast<std::chrono::milliseconds> (fall_end - fall_start);
//       string s = to_string((int)milli.count());

//       ofstream fout;
//       fout.open("/home/ubuntu/result.txt", ios::out | ios::app);
//       if (!fout) {
//         ROS_INFO("Open Failed");
//       }
//       fout << s << "\n";
//       ROS_INFO("Save completed");
//       fout.close();
//       freefalled = false;
//       start_set = false;
//       test.testing(0.172);
//     }
//   }
// }


int main(int argc, char **argv){
  wiringPiSetupGpio(); // Initalize Pi GPIO
  while(1){
    char * result = read_result();

    for (int i=0; i < 27; i++){
      std::cout << result[i] << " ";
    }
    std::cout << std::endl;
  }


  // ros::init(argc, argv, "motor_control");
  // ros::NodeHandle nh;

  // ros::Subscriber sub = nh.subscribe("/imu/data",100,msgCallback);

  // ros::spin();



  return 0;
}
