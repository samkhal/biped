// Simple node for checking heart beat and more
#include <lcm/lcm-cpp.hpp>
#include "biped_lcm/commData2Teensy.hpp"
#include "common/serial_channels.hpp"
#include "biped_lcm/heartBeat.hpp"
#include "biped_lcm/heartBeatResponse.hpp"
#include <iostream>
#include <unistd.h>

using namespace biped_lcm;
std::vector<std::string> prefixes = {"UR_","UL_","LR_","LL_"};
const int teensies = 4;
bool heartBeatFlag[teensies] = {false,false,false,false};
bool communicationInitialized[teensies] = {false,false,false,false};
unsigned int counter[teensies] = {0,0,0,0};
const unsigned int maxCount = 150;

//=========================CALLBACKS=================================
void heartBeatListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const heartBeat* msg,
					void* context){
	heartBeatFlag[msg->teensy] = true;
}

void cmdListener0(const lcm::ReceiveBuffer* rbuf,const std::string& channel,const commData2Teensy* msg,void* context){
  communicationInitialized[0] = true;
  std::cout << "Communication Initialized 0"<< std::endl;
}

void cmdListener1(const lcm::ReceiveBuffer* rbuf,const std::string& channel,const commData2Teensy* msg,void* context){
  communicationInitialized[1] = true;
  std::cout << "Communication Initialized 1"<< std::endl;
}

void cmdListener2(const lcm::ReceiveBuffer* rbuf,const std::string& channel,const commData2Teensy* msg,void* context){
  communicationInitialized[2] = true;
  std::cout << "Communication Initialized 2"<< std::endl;
}

void cmdListener3(const lcm::ReceiveBuffer* rbuf,const std::string& channel,const commData2Teensy* msg,void* context){
  communicationInitialized[3] = true;
  std::cout << "Communication Initialized 3"<< std::endl;
}


//=======================MAIN=======================================

int main(){
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

  lcm.subscribeFunction("UR_cmd_in", &cmdListener0, (void*)nullptr);
	lcm.subscribeFunction("UR_heartbeat", &heartBeatListener, (void*)nullptr);
	lcm.subscribeFunction("UL_cmd_in", &cmdListener1, (void*)nullptr);
	lcm.subscribeFunction("UL_heartbeat", &heartBeatListener, (void*)nullptr);
	lcm.subscribeFunction("LR_cmd_in", &cmdListener2, (void*)nullptr);
	lcm.subscribeFunction("LR_heartbeat", &heartBeatListener, (void*)nullptr);
	lcm.subscribeFunction("LL_cmd_in", &cmdListener3, (void*)nullptr);
	lcm.subscribeFunction("LL_heartbeat", &heartBeatListener, (void*)nullptr);

	while(lcm.good()){
		lcm.handleTimeout(0);
    usleep(1000);
		for (int i = 0; i<teensies;i++ ){
			if (communicationInitialized[i]){
	      if (heartBeatFlag[i]){
	        heartBeatResponse msgOut;
	        msgOut.beat = true;
	        lcm.publish(prefixes[i]+"heartbeatresponse", &msgOut);
	        heartBeatFlag[i] = false;
	        counter[i] = 0;
	      }
	      counter[i]++;
	      if (counter[i]>=maxCount){
	        std::cout << "WATCHDOG TIMER ERROR, TEENSY DIDN'T RESPOND"+i<< std::endl;
	        communicationInitialized[i] = false;
	      }
	    }
		}
	}
	return 0;
}
