#include <iostream>
#include "bridge.hpp"
#include "biped_lcm/blink_command.hpp"
#include "biped_lcm/blink_count.hpp"

// Open a bridge for passing the blink messages around
// TODO: global storage of channel IDs

int main(){
	LCMSerialBridge bridge("/dev/ttyACM0");
	std::cout << "Opened port" << std::endl;
	bridge.add_subscriber(0, "BLINK_COMMAND");
	bridge.add_publisher<biped_lcm::blink_count>(1, "BLINK_COUNT");

	while(1){
		bridge.handle(1);
		bridge.process_serial();
		usleep(1000); //1 ms
	}
	return 0;
}