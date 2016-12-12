#include <iostream>
#include "bridge.hpp"

#include "common/serial_channels.hpp"
#include "biped_lcm/log_msg.hpp"
#include "biped_lcm/robot_state.hpp"

// Open a bridge for passing the blink messages around
// TODO: global storage of channel IDs

int main(){
	std::string port = "/dev/ttyACM0";
	std::string prefix = "teensy_ul_";
	LCMSerialBridge bridge(port);
	std::cout << "Opened port " << port << std::endl;

	bridge.add_subscriber(CMD_MODE, prefix+"cmd_mode");
	bridge.add_subscriber(CMD_POS, prefix+"cmd_pos");

	bridge.add_publisher<biped_lcm::log_msg>(LOG_MSG, prefix+"log_msg");
	bridge.add_publisher<biped_lcm::robot_state>(STATE, prefix+"state");

	while(1){
		bridge.handle(1);
		bridge.process_serial();
		usleep(1000); //1 ms
	}
	return 0;
}