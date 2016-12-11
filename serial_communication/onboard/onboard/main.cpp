#include "WProgram.h"
#include <EEPROM.h> //ROM memory library
#include <TimerThree.h> // Timer library
#include "slave_bridge.hpp" //LCM slave file
#include "commData2Teensy.hpp" // header file for data from dispatcher to teensy
#include "commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
#include "Joint.h" // struct that stores joint data
#include "ConstJoint.h" // struct for constant joint data
#include "ROM_DATA.h" // array to data addresses and enum

LCMSerialSlave lcm; //initialize LCM object

//PID Variables
uint16_t minPot;
uint16_t maxPot;

//State Machine Variables
volatile unsigned long int timer = 0;
volatile bool allowPD = true; //flag for allowing the control


//takes number of link/joint of teensy and an attribute
// attributes: 0 - zeroTheta, 1 - minPot, 2 - maxPot, 3 - orientation
int readROM(int link, int attribute) {
  int address1 = ROM[(link*2-1) + attribute*6 + 2];
  int address2 = ROM[(link*2) + attribute*6 + 2];
  int val = EEPROM.read(address1);
  val = val + (EEPROM.read(address2) << 8);
  return val;
}

void setOrientROM(ConstJoint* cjoint){
  EEPROM.write(ROM[20+cjoint->link*2-1], (byte)(cjoint->direction));
  EEPROM.write(ROM[20+cjoint->link*2], (byte)0);
}

void setZeroThetaROM(ConstJoint* cjoint){
  int val = analogRead(cjoint->position);
  EEPROM.write((2+cjoint->link*2-1), (byte)(val));
  EEPROM.write((2+cjoint->link*2), (byte)(val >> 8));
}

ConstJoint linksConst[3] = {
  JointTable[EEPROM.read(ROM_ENUM::link1Addr)], //make enums
  JointTable[EEPROM.read(ROM_ENUM::link2Addr)],
  JointTable[EEPROM.read(ROM_ENUM::link3Addr)]};

Joint links [3];

Joint joint;
Joint* joint_p;
ConstJoint cjoint;
ConstJoint* cjoint_p;

//Writes the the maximum and minimum pot values to ROM,
//takes a pointer of constJoint struct, returns false if out of range
bool Calibration(ConstJoint* cjoint) {
  int potVal = analogRead(cjoint->position);
  if (minPot > (uint16_t)potVal) {
    minPot = (uint16_t)potVal;
  }
  else if (maxPot < (uint16_t)potVal) {
    maxPot = (uint16_t)potVal;
  }
  cjoint->minPot = minPot;
  cjoint->maxPot = maxPot;
  EEPROM.write(8+cjoint->link*2-1, (byte)(cjoint->minPot));
  EEPROM.write(8+cjoint->link*2, (byte)(cjoint->minPot >> 8));
  EEPROM.write(14+cjoint->link*2-1, (byte)(cjoint->maxPot));
  EEPROM.write(14+cjoint->link*2, (byte)(cjoint->maxPot >> 8));

  if (potVal < 10 || potVal > 1014) {
    return false;
  }
  return true;
}

//Timer interrupt callback, increases the timer variable and sets the flag ready for PID control
void timerCallback() {
  timer++;
  if (timer >= 10) { //every 10 ms
    allowPD = true;
    timer = 0;
  }
}

void callback(CHANNEL_ID id, commData2Teensy* msg_IN){
  commDataFromTeensy msg_OUT;
  msg_OUT.joint1 = msg_IN->command;
  lcm.publish(1, &msg_OUT);
}


//============================ MAIN LOOP ================================

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
