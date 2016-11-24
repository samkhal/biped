#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "arduino-serial-lib.h"

int main(){
	printf("Start\n");
	int serial = serialport_init("/dev/ttyACM0", 115200);

	// Write a character to the Teensy
	char send[] = "t";
	write(serial, send, 1);
	printf("Sent: %c\n", send[0]);

	// Wait for the echo
	char out[1];
	int n;
	do{
		n = read(serial, out, 1);
		if(n==-1){
			printf("failed to read\n");
			return -1;
		}
		usleep(1000); //wait 1 ms
	} while (n<1);

	printf("Received: %c\n",out[0]);

	close(serial);
}