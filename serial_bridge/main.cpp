#include <iostream>
#include "bridge.hpp"
#include "biped_lcm/commData2Teensy.hpp"
#include "biped_lcm/commDataFromTeensy.hpp"
#include "biped_lcm/error_channel.hpp"

// Open a bridge for passing the blink messages around
// TODO: global storage of channel IDs

typedef ChannelID CID;

int main(){
	std::string port = "/dev/ttyACM0";
	std::string prefix = "teensy_ul_";
	LCMSerialBridge bridge(port);
	std::cout << "Opened port " << port << std::endl;

	bridge.add_subscriber(CID::CMD_MODE, prefix+"cmd_mode");

	bridge.add_publisher<biped_lcm::log_msg>(CID::LOG_MSG, prefix+"log_msg");
	bridge.add_publisher<biped_lcm::commDataFromTeensy>(CID::STATE, prefix+"state");

	while(1){
		bridge.handle(1);
		bridge.process_serial();
		usleep(1000); //1 ms
	}
	return 0;
}
