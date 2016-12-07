#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

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
	return 0;
}
