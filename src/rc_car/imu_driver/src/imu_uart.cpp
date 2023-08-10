#include "imu_driver/imu_uart.hpp"

#include <errno.h>  //Error integer and strerror() function
#include <fcntl.h>  // Contains file controls like O_RDWR
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>
#include <stdbool.h>
#include <stdlib.h>
#include <termios.h>  // Contains POSIX terminal control definitions
#include <unistd.h>   // write(), read(), close()

#include <cstdio>

sensor_msgs::Imu parse_imu(char* data) {
  sensor_msgs::Imu parsed_data;
  char rpy[13][6] {0}; // initialize to 0
  int digit = 0;
  int j = 0;
  for (int i=0; i<30; i++) {
    if (data[i + 8] == ',') { // starts from i + 8 to discard first few characters
      j++;
      digit = 0;
      continue;
    }
    if (data[i + 8] == '*') break; // end of rpy data
    rpy[j][digit] = data[i + 8];
    digit++;
  }

  double roll = atof(rpy[0]); // roll
  double pitch = atof(rpy[1]); // pitch
  double yaw = atof(rpy[2]); // yaw
  tf::Quaternion orientation = tf::createQuaternionFromRPY(roll, pitch, yaw);

  parsed_data.orientation.x = orientation[0];
  parsed_data.orientation.y = orientation[1];
  parsed_data.orientation.z = orientation[2];
  parsed_data.orientation.w = orientation[3];

  parsed_data.linear_acceleration.x = atof(rpy[3]);
  parsed_data.linear_acceleration.y = atof(rpy[4]);
  parsed_data.linear_acceleration.z = atof(rpy[5]);

  parsed_data.angular_velocity.x = atof(rpy[6]);
  parsed_data.angular_velocity.y = atof(rpy[7]);
  parsed_data.angular_velocity.z = atof(rpy[8]);

  return parsed_data;
}


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
//   unsigned char msg[] = {0x40,0x61,0x73,0x63,0x5F,0x6F,0x75,0x74,0x2C,0x52,0x50,0x59,0x49,0x4D,0x55,0x2A,0x32,0x36,0x0D,0x0A};

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


sensor_msgs::Imu read_result() {
  char * result = (char *)malloc(sizeof(char) * 150);
  for (int j=1;j<=150;j++)
    {result[j] = 0;}

  int serial_port = open("/dev/ttyACM0", O_RDWR);
  // Create new termios struct, we call it 'tty' for convention
  struct termios tty;

  // Read in existing settings, and handle any error
  if(tcgetattr(serial_port, &tty) != 0) {
    std::printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
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
    std::printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
  }

  unsigned char msg[] = {0x40,0x61,0x73,0x63,0x5F,0x6F,0x75,0x74,0x2C,0x52,0x50,0x59,0x49,0x4D,0x55,0x2A,0x32,0x36,0x0D,0x0A};
  write(serial_port, msg, sizeof(msg));

  // Allocate memory for read buffer, set size according to your needs
  char read_buf[150]{0}; // initialize with 0

  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
    std::printf("Error reading: %s", strerror(errno));
  }

  close(serial_port);

  // return parse_imu(read_buf);
  for (int i=0; i<130;i++){
    result[i] = read_buf[i + 11];
  }
  if (result[0] != ',') {
    return parse_imu(result);
  }
  else {
    sensor_msgs::Imu empty;
    empty.header.seq = -1;
    return empty;
  }
}
