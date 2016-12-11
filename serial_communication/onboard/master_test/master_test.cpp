// Simple node for publishing and listening to blink messages
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include "commData2Teensy.hpp" // header file for data from dispatcher to teensy
#include "commDataFromTeensy.hpp" // header file for data from teensy to dispatcher

// Print the blink count when message received
void echoListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const commDataFromTeensy* msg,
					void* context){
	std::cout << "Received command: " << msg->joint1 << std::endl;
}

int main(){
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	lcm.subscribeFunction("DATA_FROM_TEENSY", &echoListener, (void*)nullptr);

	std::cout << "Publishing test command!" << std::endl;

	commData2Teensy msg;
	msg.command = commData2Teensy::RUN_STATIC_ALL;
	lcm.publish("DATA_2_TEENSY", &msg);
	while(0 == lcm.handle());

	return 0;
}
