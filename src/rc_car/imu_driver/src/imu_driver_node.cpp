#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

#include "imu_driver/imu_uart.hpp"

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_driver");
  ros::NodeHandle n;
  ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu>("/imu/data", 1000);

  while (ros::ok()) {
    sensor_msgs::Imu imu_data = read_result();
    if (imu_data.header.seq != -1) {
      imu_pub.publish(imu_data);
    }

    ros::spinOnce();
  }

  return 0;
}