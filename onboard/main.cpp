#include "WProgram.h"
#include <TimerThree.h> // Timer library
#include "slave_bridge.hpp" //LCM slave file
#include "biped_lcm/commData2Teensy.hpp" // header file for data from dispatcher to teensy
#include "biped_lcm/commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
#include "Joint.h" // struct that stores joint data
#include "JointTable.h"
#include "common/serial_channels.hpp"
#include "biped_lcm/log_msg.hpp"
#include "log_util.hpp"

using namespace biped_lcm; // for messages

const int numOfJoints = 3;

LCMSerialSlave lcm; //initialize LCM object
Logger loginfo(lcm, log_msg::INFO);
Logger logwarn(lcm, log_msg::WARN);
Logger logerr(lcm, log_msg::ERROR);

std::vector<JointROM> jointMem; // initialize array of 3 ROM memory structs
std::vector<Joint> joints; //vector of joints

//State Machine Variables
enum STATES{
  CALIBRATION,
  RECEIVE_TRAJECTORY,
  RUN_STATIC_CONTROL,
  RUN_STATIC_ALL,
  RUN_TRAJECTORY,
  RUN_ALL_TRAJECTORIES,
  WAIT
};
int state = STATES::WAIT;
int currLocalJoint = 0;
volatile unsigned long int timer = 0;

//Stops motors, used also as emergency stop
void stopMotors(){
  for (int i=0; i<numOfJoints; i++){
    joints[i].motorPWM(zeroTorque);
    joints[i].setEnable(LOW);
  }
}

//Convert from Joint number 1-12 to 0-2 for local uses
int convertToLocalJointNumber(int num){
  return ((num-1)%numOfJoints);
}

//Timer interrupt callback does nothing for now
void timerCallback() {
  timer++;
}

//=============================State Assignment functions=================================
void ID_Request(){
  commDataFromTeensy msgOut;
  for (int i = 0; i<numOfJoints; i++){
    msgOut.joints[i] = (uint8_t) joints[i].getJointNumber();
  }
  lcm.publish(ChannelID::STATE, &msgOut);
}

// //==============================State Machine functions===================================
void Calibration_State(){
  if (joints[currLocalJoint].CalibrationCheck() == false) {
    error_channel error_msg;
    // error_msg.name = "potentiometer hardware out of range";
    lcm.publish(ChannelID::LOG_MSG, &error_msg);
    state = STATES::WAIT;
  }
}

void Static_Control_State(){
  bool OutOfRange = joints[currLocalJoint].checkOOR();
  if (OutOfRange) {
    stopMotors();
    error_channel error_msg;
    // error_msg.name = "out of range";
    lcm.publish(LOG_MSG, &error_msg);
    state = STATES::WAIT;
  }
  else{
    joints[currLocalJoint].PIDcontrol();
  }
}

void Static_Control_All_State(){
  bool OutOfRange = joints[0].checkOOR()||joints[1].checkOOR()||joints[2].checkOOR();
  if (OutOfRange) {
    stopMotors();
    error_channel error_msg;
    // error_msg.name = "out of range";
    lcm.publish(ChannelID::LOG_MSG, &error_msg);
    state = STATES::WAIT;
  }
  else{
    for (int i=0; i<numOfJoints; i++){
      joints[i].PIDcontrol();
    }
  }
}

//================================STATE_ASSIGNMENT=======================================
//State assignment
void stateAssignment(int command){
  switch (command) {

    case commData2Teensy::ID_REQUEST:
      if (state == STATES::WAIT){
        ID_Request();
      }
      else{
        error_channel msg_Out;
        // msg_Out.name = "inappropriate command";
        lcm.publish(ChannelID::LOG_MSG, &msg_Out);
      }
      break;

    case commData2Teensy::START_CALIBRATION:
      if (state == STATES::WAIT){
        uint16_t val = (uint16_t)joints[currLocalJoint].readPotentiometer();
        joints[currLocalJoint].setMinPot(val);
        joints[currLocalJoint].setMaxPot(val);
        joints[currLocalJoint].setZeroPot(val);
        state = STATES::CALIBRATION;
      }
      else{
        error_channel msg_Out;
        // msg_Out.name = "inappropriate command";
        lcm.publish(ChannelID::LOG_MSG, &msg_Out);
      }
      break;

    case commData2Teensy::STOP_CALIBRATION:
      if (state == STATES::CALIBRATION){
        commDataFromTeensy msg_Out;
        msg_Out.minPot = joints[currLocalJoint].getMinPot();
        msg_Out.maxPot = joints[currLocalJoint].getMaxPot();
        joints[currLocalJoint].writeROM_potRange();
        lcm.publish(ChannelID::STATE, &msg_Out);
        state = STATES::WAIT;
      }
      else{
        error_channel msg_Out;
        // msg_Out.name = "inappropriate command";
        lcm.publish(ChannelID::LOG_MSG, &msg_Out);
      }
      break;

    case commData2Teensy::GET_CALIBRATION:
      if (state == STATES::WAIT){
        commDataFromTeensy msg_Out;
        msg_Out.minPot = joints[currLocalJoint].getMinPot();
        msg_Out.maxPot = joints[currLocalJoint].getMaxPot();
        lcm.publish(ChannelID::STATE, &msg_Out);
      }
      else{
        error_channel msg_Out;
        // msg_Out.name = "inappropriate command";
        lcm.publish(ChannelID::LOG_MSG, &msg_Out);
      }
      break;

    case commData2Teensy::RECEIVE_TRAJECTORY:
      break;

    case commData2Teensy::RUN_STATIC_CONTROL:
      if (state == STATES::WAIT){
        joints[currLocalJoint].setSetPointFromPot();
        joints[currLocalJoint].setEnable(HIGH);
        state = commData2Teensy::RUN_STATIC_CONTROL;
      }
      else{
        error_channel msg_Out;
        // msg_Out.name = "inappropriate command";
        lcm.publish(ChannelID::LOG_MSG, &msg_Out);
      }
      break;

    case commData2Teensy::RUN_STATIC_ALL:
      if (state == STATES::WAIT){
        for (int i=0; i<3; i++){
          joints[i].setSetPointFromPot();
          joints[i].setEnable(HIGH);
        }
        state = commData2Teensy::RUN_STATIC_ALL;
      }
      else{
        error_channel msg_Out;
        // msg_Out.name = "inappropriate command";
        lcm.publish(ChannelID::LOG_MSG, &msg_Out);
      }
      break;

    case commData2Teensy::RUN_TRAJECTORY:
      break;

    case commData2Teensy::RUN_ALL_TRAJECTORIES:
      break;

    case commData2Teensy::STOP:
      stopMotors();
      state = STATES::WAIT;
      break;

  }
}

//============================CALLBACK====================================
void callback(CHANNEL_ID id, commData2Teensy* msg_IN){
  int command = msg_IN->command;
  int currJoint = msg_IN->joint;
  currLocalJoint = convertToLocalJointNumber(currJoint);
  stateAssignment(command);
}


//============================ MAIN LOOP ================================

// Setup - Runs once
void setup() {
  Serial.begin(115200);
  ROM_allocate(numOfJoints, jointMem);
  for (int i=0; i<numOfJoints; i++){
    // EEPROM.put(jointMem[i].jointAddr, (uint8_t)(i)); // In case we want to reassign ROM joint number
    uint8_t index;
    EEPROM.get(jointMem[i].jointAddr,index);
    joints.push_back(JointTable[i]);
    joints[i].setSetPointFromPot();
    joints[i].setMemoryAddr(jointMem[i]);
    pinMode(joints[i].getMotorPin(), OUTPUT);
    pinMode(joints[i].getEnablePin(), OUTPUT);
    joints[i].motorPWM(zeroTorque);
    joints[i].setEnable(LOW);
    joints[i].setLocalJointNum();
    joints[i].writeROM_orientation();
    joints[i].writeROM_zeroTheta();
  }
  Timer3.initialize(1000); //1 ms
  Timer3.attachInterrupt(timerCallback);
  lcm.subscribe(ChannelID::CMD_MODE, &callback); // 0 is incoming, 1 is out
}

// Main loop
void loop() {
  lcm.handle();
  switch (state) {

    case STATES::CALIBRATION:
      Calibration_State();
      break;

    case STATES::RUN_STATIC_CONTROL :
      Static_Control_State();
      break;

    case STATES::RUN_STATIC_ALL :
      Static_Control_All_State();
      break;

    case STATES::RUN_TRAJECTORY :
    break;

    case STATES::RUN_ALL_TRAJECTORIES :
    break;

    case STATES::WAIT :
      break;

    default:
      break;
  }
}

int main(){
  setup();
  while(1){
    loop();
  }
  return 0;
}
