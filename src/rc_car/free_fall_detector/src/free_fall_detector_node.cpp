#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class FreeFallDetector {
 public:
  FreeFallDetector() {
    // Topic you want to publish
    pub_ = n_.advertise<sensor_msgs::Imu>("/imu_pub", 1000);

    // Topic you want to subscribe
    sub_ = n_.subscribe("/imu/data", 1000, &FreeFallDetector::callback, this);
  }

  void callback(const sensor_msgs::Imu::ConstPtr& msg) { pub_.publish(msg); }

 private:
  ros::NodeHandle n_;
  ros::Publisher pub_;
  ros::Subscriber sub_;

};  // End of class FreeFallDetector

int main(int argc, char** argv) {
  ros::init(argc, argv, "free_fall_detector");
  FreeFallDetector free_fall_detector;

  ros::spin();

  return 0;
}