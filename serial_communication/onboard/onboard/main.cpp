#include "WProgram.h"
#include <EEPROM.h> //ROM memory library
#include <TimerThree.h> // Timer library
#include "slave_bridge.hpp" //LCM slave file
#include "commData2Teensy.hpp" // header file for data from dispatcher to teensy
#include "commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
#include "Joint.h" // struct that stores joint data
#include "ConstJoint.h" // struct for constant joint data

LCMSerialSlave lcm; //initialize LCM object

void callback(CHANNEL_ID id, commData2Teensy* msg_IN){
  commDataFromTeensy msg_OUT;
  msg_OUT.joint1 = msg_IN->command;
  lcm.publish(1, &msg_OUT);
}

// Setup - Runs once
void setup() {
  Serial.begin(115200);
  lcm.subscribe(0, &callback);
}

// Main loop
void loop() {
  lcm.handle();
}

int main(){
  setup();
  while(1){
    loop();
  }
  return 0;
}
