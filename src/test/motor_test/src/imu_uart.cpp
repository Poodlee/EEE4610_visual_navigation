#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <unistd.h> // write(), read(), close()
#include <termios.h> // Contains POSIX terminal control definitions
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> //Error integer and strerror() function

#include "motor_test/imu_uart.hpp"

// For list return, we use pointer
char * setting() {

 char * result = (char *)malloc(sizeof(char) * 150);
 for (int j=1;j<=150;j++)
  result[j] = 0;

  int serial_port = open("/dev/ttyUSB0", O_RDWR);
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

  unsigned char checksum = 0;
 // Transmittion data (3rd item: servo ID)
  unsigned char msg[] = { 0x40,0x76,0x65,0x72,0x73,0x69,0x6F,0x6E,0x2A,0,0x0D,0x0A};

  for (int i=0; i<9;i++){
    checksum += msg[i];
  }
  // 6. Buffer Writing
  checksum = ~ checksum;
  msg[9] = checksum;

  // 7. Transmission code
  write(serial_port, msg, sizeof(msg));
  // Allocate memory for read buffer, set size according to your needs
  char read_buf [150];

  int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));

  // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
  if (num_bytes < 0) {
      printf("Error reading: %s", strerror(errno));
  }

  for (int i=0; i<150;i++){
    result[i] = read_buf[i];
  }


  // while(1){
  //   if (read_buf[0] == 85 && read_buf[1] == 85 && read_buf[2] == i && read_buf[4] == 26 && read_buf[5] < 100){
  //     temp_result[i-1] = read_buf[5];
  //     break;
  //   }
  //   else{
  //     write(serial_port, msg, sizeof(msg));
  //     num_bytes = read(serial_port,&read_buf,sizeof(read_buf));
  //   }
  // }

  close(serial_port);


 return result;
}



char * read_result(){

}