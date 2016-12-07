#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

#include "arduino-serial-lib.h"
#include <SerialStream.h>
#include <iostream>

using byte = unsigned char;

int main(){
	LibSerial::SerialStream serial;
	serial.Open("/dev/ttyACM0");
	serial.SetBaudRate(LibSerial::SerialStreamBuf::BAUD_115200);
	serial.unsetf( std::ios_base::skipws ) ;

	for(unsigned int i=0;i<256;i++){
		char msg2 = i;
		std::cout<< "Sending "<<(int)msg2 <<std::endl;

		serial << msg2;

		unsigned char recv;
		serial >> recv;
		std::cout<<"Received "<<(int)recv<<std::endl;
	}
	char msg2 = 1;
	std::cout<< "Sending"<<(int)msg2 <<std::endl;

	serial << msg2;

	unsigned char recv;
	serial >> recv;
	std::cout<<(int)recv<<std::endl;
	return 0;


}

int simple_serial_echo(){
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