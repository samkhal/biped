#include <iostream>
#include "bridge.hpp"
#include "commData2Teensy.hpp" // header file for data from dispatcher to teensy
#include "commDataFromTeensy.hpp" // header file for data from teensy to dispatcher

// Open a bridge for passing the blink messages around
// TODO: global storage of channel IDs

int main(){
	LCMSerialBridge bridge("/dev/ttyACM0");
	std::cout << "Opened port" << std::endl;
	bridge.add_subscriber(0, "DATA_FROM_TEENSY");
	bridge.add_publisher<commData2Teensy>(1, "DATA_2_TEENSY");

	while(1){
		bridge.handle(1);
		bridge.process_serial();
		usleep(1000); //1 ms
	}
	return 0;
}
