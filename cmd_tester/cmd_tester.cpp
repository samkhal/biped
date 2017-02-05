// Simple node for publishing and listening to blink messages
#include <lcm/lcm-cpp.hpp>
#include "biped_lcm/commData2Teensy.hpp"
#include "biped_lcm/commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
#include "biped_lcm/LiveControl2Teensy.hpp" //header file for live messages
#include "biped_lcm/LiveControlFromTeensy.hpp" //header file for live messages
#include "common/serial_channels.hpp"
#include "biped_lcm/log_msg.hpp"
#include <iostream>

using namespace biped_lcm;

void logMsgListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const log_msg* msg,
					void* context){
	std::cout << msg->msg << std::endl;
}

void cmdResponseListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const commDataFromTeensy* msg,
					void* context){
	std::cout << "Received command: " << msg->joints[0] << std::endl;
	std::cout << "Received command: " << msg->joints[1] << std::endl;
	std::cout << "Received command: " << msg->joints[2] << std::endl;
	std::cout << "Received command: " << msg->minPot << std::endl;
	std::cout << "Received command: " << msg->maxPot << std::endl;
}

void liveControlListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const LiveControlFromTeensy* msg,
					void* context){
	// std::cout << "Joint: " << msg->joint << std::endl;
	std::cout << "Current: " << msg->current << std::endl; //Just printing timer for test
	// std::cout << "Angle: " << msg->angle << std::endl;
}

int main(){

	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	while(lcm.good()){
		commData2Teensy msg;
		std::cin >> msg.command;
		std::cout << "Publishing command " << msg.command << std::endl;
		lcm.publish("UL_cmd_mode", &msg);

	lcm.subscribeFunction("cmd_response", &cmdResponseListener, (void*)nullptr);
	lcm.subscribeFunction("log_msg", &logMsgListener, (void*)nullptr);
	lcm.subscribeFunction("live_out", &liveControlListener, (void*)nullptr);

	commData2Teensy msg;
	std::cin >> msg.command;
	std::cout << "Publishing command " << msg.command << std::endl;
	lcm.publish("cmd_in", &msg);
	while(lcm.good()){
		lcm.handle();
		commData2Teensy msg;
		std::cin >> msg.command;
		std::cout << "Publishing command " << msg.command << std::endl;
		lcm.publish("cmd_in", &msg);
	}
	return 0;
}
