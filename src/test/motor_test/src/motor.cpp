// motor.cpp
#include "motor.hpp"
// #include "imu_uart.hpp"
#include <unistd.h>

#include <chrono>
#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <sstream>

#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h> // write(), read(), close()
#include <termios.h> // Contains POSIX terminal control definitions
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> //Error integer and strerror() function

// #include "ros/ros.h"
// #include "sensor_msgs/Imu.h"  // Add Topic msg header

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












// char * setting() {

//  char * result = (char *)malloc(sizeof(char) * 150);
//  for (int j=1;j<=150;j++)
//   result[j] = 0;

//   int serial_port = open("/dev/ttyACM0", O_RDWR);
//   // Create new termios struct, we call it 'tty' for convention
//   struct termios tty;

//   // Read in existing settings, and handle any error
//   if(tcgetattr(serial_port, &tty) != 0) {
//       printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
//   }

//   tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
//   tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
//   tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
//   tty.c_cflag |= CS8; // 8 bits per byte (most common)
//   tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
//   tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

//   tty.c_lflag &= ~ICANON;
//   tty.c_lflag &= ~ECHO; // Disable echo
//   tty.c_lflag &= ~ECHOE; // Disable erasure
//   tty.c_lflag &= ~ECHONL; // Disable new-line echo
//   tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
//   tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
//   tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

//   tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
//   tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
//   // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
//   // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

//   tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
//   tty.c_cc[VMIN] = 0;

//   // Set in/out baud rate to be 9600
//   cfsetispeed(&tty, B115200);
//   cfsetospeed(&tty, B115200);

//   // Save tty settings, also checking for error
//   if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
//     printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
//   }

//   unsigned char checksum = 0;
//  // https://codebeautify.org/xor-calculator ( ~Front 2A)
//  // Capital? lower?
//   unsigned char msg[] = { 0x40,0x62,0x61,0x75,0x64,0x72,0x61,0x74,0x65,0x2A,0x35,0x30,0x0D,0x0A};

//   // 7. Transmission code
//   write(serial_port, msg, sizeof(msg));
//   // Allocate memory for read buffer, set size according to your needs
//   char read_buf [150];

//   int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

//   // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
//   if (num_bytes < 0) {
//       printf("Error reading: %s", strerror(errno));
//   }

//   for (int i=0; i<150;i++){
//     result[i] = read_buf[i];
//   }
//   return result;
// }

char * read_result(){
 char * result = (char *)malloc(sizeof(char) * 150);
 for (int j=1;j<27;j++)
  result[j] = 0;

  int serial_port = open("/dev/ttyACM0", O_RDWR);
  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
  }

  tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
  tty.c_cflag &= ~CSIZE; // Clear all bits that set the data size
  tty.c_cflag |= CS8; // 8 bits per byte (most common)
  tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO; // Disable echo
  tty.c_lflag &= ~ECHOE; // Disable erasure
  tty.c_lflag &= ~ECHONL; // Disable new-line echo
  tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
  tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
  tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
  // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
  // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

  tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
  cfsetispeed(&tty, B115200);
  cfsetospeed(&tty, B115200);

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  // Allocate memory for read buffer, set size according to your needs
  char read_buf [27];

  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
  }

  for (int i=0; i<150;i++){
    result[i] = read_buf[i];
  }


  close(serial_port);


 return result;
}




int main(int argc, char **argv){
  wiringPiSetupGpio(); // Initalize Pi GPIO
  // float trans;
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
