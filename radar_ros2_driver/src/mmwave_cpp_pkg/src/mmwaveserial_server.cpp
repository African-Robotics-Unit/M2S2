// C library headers
#include <stdio.h>
#include <string.h>
#include <chrono>
#include <thread>
#include <time.h>

// Linux headers
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> // write(), read(), close()
#include <sys/types.h>
#include <stdlib.h>
#include <sys/ioctl.h>

// Definitions
#define MAX_RESPONSE_BYTES	24
#define SERIAL_DEVICE	"/dev/ttyACM0"
#define MAX_SERIAL_TIMEOUT_NS 999999999
#define     SERIAL_TIMEOUT_NS 990000000 


// Structs
struct timespec diff(struct timespec start, struct timespec end)
{
	struct timespec temp;
	if ((end.tv_nsec-start.tv_nsec)<0) {
		temp.tv_sec = end.tv_sec-start.tv_sec-1;
		temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
	} else {
		temp.tv_sec = end.tv_sec-start.tv_sec;
		temp.tv_nsec = end.tv_nsec-start.tv_nsec;
	}
	return temp;
}

// name space definitions 
using namespace std::this_thread; // sleep_for, sleep_until
using namespace std::chrono; // nanoseconds, system_clock, seconds

int main() {

  int retval;
  long int timeout = SERIAL_TIMEOUT_NS;
  struct timespec start_time;
  struct timespec end_time;	
  struct timespec diff_time;	

  char read_buf [128];
  memset(&read_buf, '\0', sizeof(read_buf));


  // Open the serial port. -------------------------------------------------------------------------
  int serial_port = open(SERIAL_DEVICE, O_RDWR);
  if(serial_port <0){
    printf("Error %i from open: %s\n", errno, strerror(errno));
    exit(1);
  }

  // Setup port. ----------------------------------------------------------------------------------- 
  struct termios tty;

  // Read in existing settings, and handle any error.
  if(tcgetattr(serial_port, &tty) != 0) {
      printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
      close(serial_port);
      exit(2);
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

  tty.c_cc[VTIME] = 30;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
  tty.c_cc[VMIN] = 0;

  // Set in/out baud rate to be 9600
	retval = cfsetospeed(&tty, B115200);
	if (retval < 0) {
		perror("Failed to set B115200 output baud rate");
    close(serial_port);
		exit(3);
	}
	retval = cfsetispeed(&tty, B115200);
	if (retval < 0) {
		perror("Failed to set B115200 input baud rate");
    close(serial_port);
		exit(4);
	}

  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
      printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
      close(serial_port);
      exit(5);
  }

  // Flush read and write buffers
  tcflush(serial_port, TCIOFLUSH);

  // Write to serial port. ---------------------------------------------------------------------------------
  unsigned char msg[] = {'d','f','e','D','a','t','a','O','u','t','p','u','t','M','o','d','e',' ','1','\n'};
  //commandBytesLen = sizeof(msg);
  retval = write(serial_port, msg, sizeof(msg));
  if (retval < 0) {
			perror("write on SERIAL_DEVICE failed");
      close(serial_port);
			exit(6);
	}
  
  // wait till command is written
  tcdrain(serial_port);

  // Read response -----------------------------------------------------------------------------------------
  clock_gettime(CLOCK_MONOTONIC, &start_time);
  int readSuccessFlag = 0;
  while (readSuccessFlag == 0) {
		// get bytes available
    int bytes_available;
		retval = ioctl(serial_port, FIONREAD, &bytes_available);
		if (retval < 0) {
			perror("FIONREAD ioctl failed\n");
      close(serial_port);
			exit(6);
		}

    // wait 20 milliseconds
		sleep_for(milliseconds(20));
		
    // exit wait if minimum bytes read
    if (bytes_available >= (1))
    {
      printf("%d bytes available in buffer.\n",bytes_available);
      
      // n is the number of bytes read. n may be 0 if no bytes were received, and can also be -1 to signal an error.
      int num_bytes = read(serial_port, &read_buf, sizeof(read_buf));
      if (num_bytes < 0) {
        printf("Error reading: %s", strerror(errno));
        close(serial_port);
        exit(7);
      }
      printf("Read %i bytes. Received message: %s\n", num_bytes, read_buf);
		  readSuccessFlag = 1;
    }
    
    // exit wait if timeout
    clock_gettime(CLOCK_MONOTONIC, &end_time);
    //printf("%ld\n",end_time.tv_nsec);
    diff_time = diff(start_time, end_time);
    if (diff_time.tv_nsec>=timeout){
      printf("Read timed out.\n");
      readSuccessFlag = -1;
    }
	}

  close(serial_port);
  return 0; // success
};