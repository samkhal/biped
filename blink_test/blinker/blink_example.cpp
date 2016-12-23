// Simple node for publishing and listening to blink messages
#include <lcm/lcm-cpp.hpp>
#include <iostream>
#include "commData2Teensy.hpp" // header file for data from dispatcher to teensy
#include "commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
#include "error_channel.hpp"

// Print the blink count when message received
void errorListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const error_channel* msg,
					void* context){
					// std::cout << msg->name << std::endl;
}

void teensyListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const commDataFromTeensy* msg,
					void* context){
	std::cout << "Received command: " << msg->joints[0] << std::endl;
	std::cout << "Received command: " << msg->joints[1] << std::endl;
	std::cout << "Received command: " << msg->joints[2] << std::endl;
	std::cout << "Received command: " << msg->minPot << std::endl;
	std::cout << "Received command: " << msg->maxPot << std::endl;
}

int main(int argc, char* argv[]){
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	lcm.subscribeFunction("FROM_TEENSY", &teensyListener, (void*)nullptr);
	lcm.subscribeFunction("ERROR", &errorListener, (void*)nullptr);

	commData2Teensy msg;
	char *p;
	long conv = strtol(argv[1], &p, 10);
	std::cout << "Publishing test command!"<< conv << std::endl;
	msg.command = conv;
	lcm.publish("TO_TEENSY", &msg);
	while(0 == lcm.handle());

	return 0;
}
