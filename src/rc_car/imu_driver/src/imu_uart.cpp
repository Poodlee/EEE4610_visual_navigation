#include <cstdio>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include <unistd.h> // write(), read(), close()
#include <termios.h> // Contains POSIX terminal control definitions
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> //Error integer and strerror() function

#include <cstdlib> // parsing char* data

#include "imu_driver/imu_uart.hpp"
#include <sensor_msgs/Imu.h>

sensor_msgs::Imu parse_imu(char* data) {
  sensor_msgs::Imu result;
  char rpy[3][6] {0}; // initialize to 0
  int digit = 0;
  int j = 0;
  for (int i=0; i<sizeof(data); i++) {
    if (data[i] == ',') {
      j++;
      digit = 0;
      continue;
    }
    if (data[i] == '*') break; // end of rpy data
    rpy[j][digit] = data[i];
    digit++;
  }

  result.angular_velocity.x = atof(rpy[0]); // roll
  result.angular_velocity.y = atof(rpy[1]); // pitch
  result.angular_velocity.z = atof(rpy[2]); // yaw

  return result;
}

sensor_msgs::Imu read_result() {
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

  // Allocate memory for read buffer, set size according to your needs
  char read_buf[30]{0}; // initialize with 0

  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
    std::printf("Error reading: %s", strerror(errno));
  }

  close(serial_port);

  return parse_imu(read_buf);
}
