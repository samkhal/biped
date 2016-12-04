// Simple node for publishing and listening to blink messages
#include <lcm/lcm-cpp.hpp>
#include "blink_command.hpp"
#include "blink_count.hpp"
#include <iostream>

void blink_listener(const lcm::ReceiveBuffer* rbuf, 
					const std::string& channel, 
					const blink_count* msg, void* context){
	std::cout << msg->count << std::endl;
}

int main(){
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	lcm.subscribeFunction("BLINK_COUNT", &blink_listener, (void*)nullptr);

	std::cout << "Publishing blink command!" << std::endl;

	blink_command msg;
	msg.command = blink_command::ON;
	lcm.publish("BLINK_COMMAND", &msg);

	while(0 == lcm.handle());

	return 0;
}