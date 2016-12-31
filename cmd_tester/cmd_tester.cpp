// Simple node for publishing and listening to blink messages
#include <lcm/lcm-cpp.hpp>
#include "biped_lcm/commData2Teensy.hpp"
#include <iostream>

using namespace biped_lcm;

int main(){
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	while(lcm.good()){
		commData2Teensy msg;
		std::cin >> msg.command;
		std::cout << "Publishing command " << msg.command << std::endl;
		lcm.publish("teensy_ul_cmd_mode", &msg);
	}
	return 0;
}