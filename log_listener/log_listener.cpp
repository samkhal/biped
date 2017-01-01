// Simple node for publishing and listening to blink messages
#include <lcm/lcm-cpp.hpp>
#include "biped_lcm/log_msg.hpp"
#include <iostream>

using namespace biped_lcm;

// Print the blink count when message received
void log_listener(const lcm::ReceiveBuffer* rbuf, 
					const std::string& channel, 
					const log_msg* msg, 
					void* context){
	std::cout << channel << ":" << msg->log_level << ":" << msg->msg << std::endl;
}

int main(){
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	lcm.subscribeFunction("UL_log_msg", &log_listener, (void*)nullptr);

	while(0 == lcm.handle());

	return 0;
}