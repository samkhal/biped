// Simple node for publishing and listening to blink messages
#include <lcm/lcm-cpp.hpp>
#include "biped_lcm/commData2Teensy.hpp"
#include "biped_lcm/commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
#include "biped_lcm/LiveControl2Teensy.hpp" //header file for live messages
#include "biped_lcm/LiveControlFromTeensy.hpp" //header file for live messages
#include "biped_lcm/LiveControlAll.hpp"
#include "common/serial_channels.hpp"
#include "biped_lcm/log_msg.hpp"
#include <iostream>
#include <unistd.h>


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
	// std::cout << "Current: " << msg->current << std::endl; //Just printing timer for test
	// std::cout << "Angle: " << msg->angle << std::endl;
}

int main(){

	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

	lcm.subscribeFunction("UR_cmd_response", &cmdResponseListener, (void*)nullptr);
	lcm.subscribeFunction("UR_log_msg", &logMsgListener, (void*)nullptr);
	lcm.subscribeFunction("UR_live_out", &liveControlListener, (void*)nullptr);

	// commData2Teensy msg;
	// std::cin >> msg.command;
	// if (msg.command != 0){
	// 	msg.joint = 0;
	// }
	// std::cout << "Publishing command " << msg.command << std::endl;
	// lcm.publish("UR_cmd_in", &msg);
	LiveControlAll msgOut;
	int x = 0;
	std::cin >> x;
	msgOut.num_joints = 1;
	msgOut.joint_ids = {1};
	msgOut.torque = {0};
	msgOut.angle = {float(float(x)/10)};
	lcm.publish("live_in",&msgOut);
	while(lcm.good()){
		lcm.handleTimeout(0);

		// LiveControlAll msgOut;
		// msgOut.num_joints = 2;
		// msgOut.joint_ids = {1,2};
		// msgOut.torque = {0.303f,0.403f};
		// msgOut.angle = {0.52f, 0.54f};
		// lcm.publish("live_in",&msgOut);

		// lcm.publish("live_in", &msgOut);
		// LiveControl2Teensy msgOut;
	 //  	msgOut.joint = 1;
		// msgOut.torque = 0.303f; //random values
		// msgOut.angle = 0.52f;
		// lcm.publish("UR_live_in",&msgOut);
		usleep(10000);
	}
	return 0;
}
