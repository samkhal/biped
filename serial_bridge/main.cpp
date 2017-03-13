#include <iostream>
#include <string>
#include <map>
#include "bridge.hpp"
#include "biped_lcm/commData2Teensy.hpp"
#include "biped_lcm/commDataFromTeensy.hpp"
#include "biped_lcm/log_msg.hpp"
#include "biped_lcm/LiveControl2Teensy.hpp" //header file for live messages
#include "biped_lcm/LiveControlAll.hpp"
#include "biped_lcm/LiveControlFromTeensy.hpp" //header file for live messages
#include "biped_lcm/heartBeat.hpp"
#include "biped_lcm/heartBeatResponse.hpp"

// Open a bridge for passing the blink messages around
// TODO: global storage of channel IDs

using namespace biped_lcm;

typedef ChannelID CID;
std::vector<std::string> prefixes = {"UR","UL","LR","LL"};
std::map<int8_t,int8_t> joint_map = {{1,0}, //Maps joint ids to teensies (index into prefix array)
									 {2,0},
									 {3,0},
									 {4,1},
									 {5,1},
									 {6,1},
									 {7,2},
									 {8,2},
									 {9,2},
									 {10,3},
									 {11,3},
									 {12,3}};


std::string live_in_channel = "live_in";
lcm::LCM dispatcher;

void dispatchControl(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const LiveControlAll* msg,
					void* context){

	std::cout<< "p";
	for(int i=0; i<msg->num_joints; i++){
		LiveControl2Teensy new_msg;
		new_msg.joint = (((msg->joint_ids[i])-1)%3);
		new_msg.torque = msg->torque[i];
		new_msg.angle = msg->angle[i];

		std::string dest = prefixes[joint_map[msg->joint_ids[i]]]+"_"+live_in_channel;
		dispatcher.publish(dest, &new_msg);
	}
}

int main(int argc, char *argv[]){
	// arguments passed in as paths to upper-right, upper-left, lower-right, lower-left serial ports
	if(argc<2 or argc>5){
		std::cout<<"List device paths in order: UR, UL, LR, LL"<<std::endl;
		std::cout<<"Usage: "<< argv[0] << " /dev/ttyACMx /dev/ttyACMx ..." << std::endl;
		return 1;
	}

	std::vector<LCMSerialBridge*> bridges;

	for(int i=0; i<argc-1; i++){
		std::string prefix = prefixes[i];
		std::string port = argv[i+1];
		LCMSerialBridge* bridge = new LCMSerialBridge(port);
		std::cout << "Opened port " << port << " for teensy " << prefix << std::endl;

		bridge->add_subscriber(CID::CMD_IN, prefix+"_cmd_in");
		bridge->add_subscriber(CID::LIVE_IN, prefix+"_"+live_in_channel);
		bridge->add_subscriber(CID::HEARTBEATRESPONSE, prefix+"_heartbeatresponse");

		bridge->add_publisher<heartBeat>(CID::HEARTBEAT, prefix+"_heartbeat");
		bridge->add_publisher<log_msg>(CID::LOG_MSG, prefix+"_log_msg");
		bridge->add_publisher<commDataFromTeensy>(CID::CMD_RESPONSE, prefix+"_cmd_response");
		bridge->add_publisher<LiveControlFromTeensy>(CID::LIVE_OUT, prefix+"_live_out");

		bridges.push_back(bridge);
	}

	// Set up dispatcher
	if(!dispatcher.good()){
		std::cout << "LCM error" << std::endl;
		return 1;
	}

	dispatcher.subscribeFunction(live_in_channel, &dispatchControl, (void*)nullptr);

	while(1){
		for(int i=0; i<bridges.size(); i++){
			bridges[i]->handle(0);
			bridges[i]->process_serial();
		}
		dispatcher.handleTimeout(0);
		usleep(1000); //1 ms
	}
	return 0;
}
