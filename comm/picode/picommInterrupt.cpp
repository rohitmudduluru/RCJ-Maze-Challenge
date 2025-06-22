#include <lccv.hpp>
#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <termios.h>                                                         
#include <stdio.h>
#include <stdlib.h>	
#include <string.h>                                                       
#include <fcntl.h>                                                                                                               
#include <sys/types.h> 
#include <stdint.h>
#include <sys/signal.h>
#include <time.h>
#include <stdbool.h>	
#include <errno.h>
#include <cstdio>
#include <time.h> 

#define LOW 0
#define HIGH 1

int setGPIO(int pin, int hl) {
	char buf[100];

	/* if we call: system("pinctrl set 13 op dl");
	 * that sets pin 13 to low
	 * if we call: system("pinctrl set 13 op dh");
	 * that sets pin 13 to hi (dl is low, dh is high)
	 */
	 // construct system call with pin and dl/dh
	sprintf(buf, "pinctrl set %d op %s", pin, (hl == LOW) ? "dl" : "dh");
	system(buf);
	return 0;
}

using namespace cv;
using namespace std;

#define BAUDRATE B115200                                                      
#define MODEMDEVICE "/dev/ttyAMA0"

int uart0_filestream = -1;
void init() {
	uart0_filestream = open(MODEMDEVICE, O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
		exit(-1);
	}
	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

}
char readSerial() {
	if (uart0_filestream != -1)
	{
		// Read up to 1 character from the port if they are there
		unsigned char rx_buffer[1];
		int rx_length = 0;

		while (rx_length <= 0)  								//remove the while to make this non-blocking
			rx_length = read(uart0_filestream, (void*)rx_buffer, 1);		//Filestream, buffer to store in, number of bytes to read (max)

		if (rx_length < 0)
		{
			//An error occured (will occur if there are no bytes)
			std::cout << "ERROR" << std::endl;
			return -1;
		}
		else if (rx_length == 0)
		{
			//No data waiting - if we are non-blocking
			return 0;
		}
		else
		{
			//Bytes received
			std::cout << "Acknowledged" << std::endl;
			return rx_buffer[0];
		}
	}
	return -1;
}

void send(char side, char victim) {
	//----- TX BYTES -----
	char tx_buffer[20];
	sprintf(tx_buffer, "%c%c", side, victim);
	if (uart0_filestream != -1)
	{
		int err = 0;
		err = write(uart0_filestream, &tx_buffer[0], strlen(tx_buffer));		//Filestream, bytes to write, number of bytes to write
		if (err < 0)
		{
			printf("UART TX error: %d\n", err);
		}
		else {
			//printf("UART TX ok\n");
		}

	}
}

#define pin 4

int iter = 8;
int main() {
	setGPIO(pin, LOW);
	init();

	while (1) {
		struct timespec remaining, request = { 0, 1000 };
		setGPIO(pin, HIGH);
		nanosleep(&request, &remaining);
		setGPIO(pin, LOW);
		cout << "Pulsed interrupt" << endl;
		sleep(1);
		send(iter / 4 + '0', iter % 4 + '0');
		cout << "Sent value: " << (char)(iter / 4 + '0') << (char)(iter % 4 + '0') << endl;
		sleep(3);
		iter++;
	}
}
