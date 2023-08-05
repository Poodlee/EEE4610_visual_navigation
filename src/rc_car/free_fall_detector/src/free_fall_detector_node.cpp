#include "interface/FreeFall.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"

class FreeFallDetector {
 public:
  FreeFallDetector() {
    pub_ = n_.advertise<interface::FreeFall>("/freefall", 1000);
    sub_ = n_.subscribe("/imu/data", 1000, &FreeFallDetector::callback, this);
  }

  void callback(const sensor_msgs::Imu::ConstPtr& msg) {
    interface::FreeFall freefall_msg;
    freefall_msg.header = msg->header;
    freefall_msg.isFreeFall = 0;

    double net_accel =
        std::sqrt((msg->linear_acceleration.x * msg->linear_acceleration.x) +
                  (msg->linear_acceleration.y * msg->linear_acceleration.y) +
                  (msg->linear_acceleration.z * msg->linear_acceleration.z));

    if (net_accel < 5.0) {
      freefall_msg.isFreeFall = 1;
    }

    pub_.publish(freefall_msg);
  }

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