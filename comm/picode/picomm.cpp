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


int main() {
	init();
	while (1) {
		for (char i = '0'; i <= '9'; i++) {
			for (char j = '0'; j <= '9'; j++) {
				send(i, j);
				cout << i << j << endl;
				sleep(2);
			}
		}
	}
}