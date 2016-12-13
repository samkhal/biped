#include <iostream>
#include "bridge.hpp"
#include "biped_lcm/commData2Teensy.hpp"
#include "biped_lcm/commDataFromTeensy.hpp"
#include "biped_lcm/error_channel.hpp"

// Open a bridge for passing the blink messages around
// TODO: global storage of channel IDs

int main(){
	LCMSerialBridge bridge("/dev/ttyACM0");
	std::cout << "Opened port" << std::endl;
	bridge.add_subscriber(0, "TO_TEENSY");
	bridge.add_publisher<biped_lcm::error_channel>(2, "ERROR");
	bridge.add_publisher<biped_lcm::commDataFromTeensy>(1, "FROM_TEENSY");

	while(1){
		bridge.handle(1);
		bridge.process_serial();
		usleep(1000); //1 ms
	}
	return 0;
}
