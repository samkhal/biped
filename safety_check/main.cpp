// Simple node for checking heart beat and more
#include <lcm/lcm-cpp.hpp>
#include "biped_lcm/commData2Teensy.hpp"
#include "common/serial_channels.hpp"
#include "biped_lcm/heartBeat.hpp"
#include "biped_lcm/heartBeatResponse.hpp"
#include <iostream>
#include <unistd.h>

using namespace biped_lcm;

bool heartBeatFlag = false;
bool communicationInitialized = false;
unsigned int counter = 0;
const unsigned int maxCount = 60;

//=========================CALLBACKS=================================
void heartBeatListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const heartBeat* msg,
					void* context){
	heartBeatFlag = true;
  std::cout << counter<< std::endl;
}

void cmdListener(const lcm::ReceiveBuffer* rbuf,
					const std::string& channel,
					const commData2Teensy* msg,
					void* context){
  communicationInitialized = true;
  std::cout << "Communication Initialized"<< std::endl;
}

//=======================MAIN=======================================

int main(){
	lcm::LCM lcm;
	if(!lcm.good())
		return 1;

  lcm.subscribeFunction("cmd_in", &cmdListener, (void*)nullptr);
	lcm.subscribeFunction("heartbeat", &heartBeatListener, (void*)nullptr);

	while(lcm.good()){
		lcm.handleTimeout(0);
    usleep(1000);
    if (communicationInitialized){
      if (heartBeatFlag){
        heartBeatResponse msgOut;
        msgOut.beat = true;
        lcm.publish("heartbeatresponse", &msgOut);
        heartBeatFlag = false;
        counter = 0;
      }
      counter++;
      if (counter>=maxCount){
        std::cout << "WATCHDOG TIMER ERROR, TEENSY DIDN'T RESPOND"<< std::endl;
        communicationInitialized = false;
      }
    }
	}
	return 0;
}
