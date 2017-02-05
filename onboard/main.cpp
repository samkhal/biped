#include "WProgram.h"
#include <TimerThree.h> // Timer library
#include "slave_bridge.hpp" //LCM slave file
#include "biped_lcm/commData2Teensy.hpp" // header file for data from dispatcher to teensy
#include "biped_lcm/commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
#include "biped_lcm/LiveControl2Teensy.hpp" //header file for live messages
#include "biped_lcm/LiveControlFromTeensy.hpp" //header file for live messages
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
Logger logerror(lcm, log_msg::ERROR);

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
int currJoint = 0;
volatile unsigned long int timer = 0;
volatile boolean PIDflag = true; //flag based on timer for PID control
volatile unsigned int timerCounter = 0;

//Stops motors, used also as emergency stop
void stopMotors(){
  for (int i=0; i<numOfJoints; i++){
    joints[i].motorPWM(zeroTorque);
    joints[i].setEnable(LOW);
  }
}

//Timer interrupt callback does nothing for now
void timerCallback() {
  timer++;
  PIDflag = true;
}

//=============================State Assignment functions=================================
void ID_Request(){
  commDataFromTeensy msgOut;
  for (int i = 0; i<numOfJoints; i++){
    msgOut.joints[i] = (uint8_t) joints[i].getJointNumber();
  }
  lcm.publish(ChannelID::CMD_RESPONSE, &msgOut);
}

boolean checkIfWaitState(){
  if (state != STATES::WAIT){
    logerror << "inappropriate command" << std::flush;
    return false;
  }
  return true;
}

// //==============================State Machine functions===================================
void Calibration_State(){
  if (joints[currJoint].CalibrationCheck() == false) {
    logerror << "potentiometer hardware out of range" << std::flush;
    state = STATES::WAIT;
  }
}

void Static_Control_State(){
  bool OutOfRange = joints[currJoint].checkOOR();
  if (OutOfRange) {
    stopMotors();
    logerror << "out of range" << std::flush;
    state = STATES::WAIT;
  }
  else{
    if (PIDflag){
      joints[currJoint].PIDcontrol();
      PIDflag = false;
    }
  }
}

void Static_Control_All_State(){
  bool OutOfRange = joints[0].checkOOR()||joints[1].checkOOR()||joints[2].checkOOR();
  if (OutOfRange) {
    stopMotors();
    logerror << "out of range" << std::flush;
    state = STATES::WAIT;
  }
  else{
    if (PIDflag){
      for (int i=0; i<numOfJoints; i++){
        joints[i].PIDcontrol();
      }
      PIDflag = false;
    }
  }
}

void Run_Trajectory_State(){
  bool OutOfRange = joints[currJoint].checkOOR();
  if (OutOfRange) {
    stopMotors();
    logerror << "OUT OF RANGE" << std::flush;
    state = STATES::WAIT;
  }
  else{
    if (PIDflag){
      joints[currJoint].PIDcontrol();
      PIDflag = false;
    }
  }
}

//================================STATE_ASSIGNMENT=======================================
//State assignment
void stateAssignment(int command){
  switch (command) {

    case commData2Teensy::ID_REQUEST:
      if (checkIfWaitState()){
        ID_Request();
      }
      break;

    case commData2Teensy::START_CALIBRATION:
      if (checkIfWaitState()){
        uint16_t val = (uint16_t)joints[currJoint].readPotentiometer();
        joints[currJoint].setMinPot(val);
        joints[currJoint].setMaxPot(val);
        joints[currJoint].setZeroPot(val);
        state = STATES::CALIBRATION;
      }
      break;

    case commData2Teensy::STOP_CALIBRATION:
      if (state == STATES::CALIBRATION){
        commDataFromTeensy msg_Out;
        msg_Out.minPot = joints[currJoint].getMinPot();
        msg_Out.maxPot = joints[currJoint].getMaxPot();
        joints[currJoint].writeROM_potRange();
        lcm.publish(ChannelID::CMD_RESPONSE, &msg_Out);
        state = STATES::WAIT;
      }
      else{
        logerror << "inappropriate command" << std::flush;
      }
      break;

    case commData2Teensy::GET_CALIBRATION:
      if (checkIfWaitState()){
        commDataFromTeensy msg_Out;
        msg_Out.minPot = joints[currJoint].getMinPot();
        msg_Out.maxPot = joints[currJoint].getMaxPot();
        lcm.publish(ChannelID::CMD_RESPONSE, &msg_Out);
      }
      break;

    case commData2Teensy::RECEIVE_TRAJECTORY:
      if (checkIfWaitState()){
        logwarn << "receive trajectory not implemented yet" << std::flush;
        state = STATES::WAIT;
      }
      break;

    case commData2Teensy::RUN_STATIC_CONTROL:
      if (checkIfWaitState()){
        joints[currJoint].setSetPointFromPot();
        joints[currJoint].setEnable(HIGH);
        state = commData2Teensy::RUN_STATIC_CONTROL;
      }
      break;

    case commData2Teensy::RUN_STATIC_ALL:
      if (checkIfWaitState()){
        for (int i=0; i<3; i++){
          joints[i].setSetPointFromPot();
          joints[i].setEnable(HIGH);
        }
        state = commData2Teensy::RUN_STATIC_ALL;
      }
      break;

    case commData2Teensy::RUN_TRAJECTORY:
      if (checkIfWaitState()){
        joints[currJoint].setSetPointFromPot(); //just in the beginning
        joints[currJoint].setEnable(HIGH);
        loginfo << "run trajectory" << std::flush;
        state = commData2Teensy::RUN_TRAJECTORY;
      }
      break;

    case commData2Teensy::RUN_ALL_TRAJECTORIES:
      break;

    case commData2Teensy::STOP:
      stopMotors();
      loginfo << "motors stopped" << std::flush;
      state = STATES::WAIT;
      break;
  }
}

//============================CALLBACK====================================
void callback(ChannelID id, commData2Teensy* msg_IN){
  int command = msg_IN->command;
  currJoint = msg_IN->joint;
  stateAssignment(command);
}

void LiveCallback(ChannelID id, LiveControl2Teensy* msg_IN){
  //send position and current, then PID control
  int torque = msg_IN->torque;
  int angle = msg_IN->angle;
  joints[msg_IN->joint].setSetPoint(angle); //get 0, 1 or 2 for joint from msgIN
  LiveControlFromTeensy msgOut;
  msgOut.joint = msg_IN->joint;
  msgOut.current = timer; // just to check the timer values at every callback
  msgOut.angle = angle;
  lcm.publish(ChannelID::LIVE_OUT, &msgOut);
}

//============================ MAIN LOOP ================================

// Setup - Runs once
void setup() {
  Serial.begin(115200);
  ROM_allocate(numOfJoints, jointMem);
  for (int i=0; i<numOfJoints; i++){
    // EEPROM.put(jointMem[i].jointAddr, (uint8_t)(i)); // In case we want to reassign ROM joint number
    uint8_t index; // this index number is from 1 to 12, representing the total joints
    EEPROM.get(jointMem[i].jointAddr,index); // get the joint number from 1 to 12 based on the jointMem vector (0-2)
    joints.push_back(JointTable[index-1]); // take the proper joint from jointTable array (0-11)
    joints[i].setSetPointFromPot(); // local joint number is from 0 to 2.
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
  lcm.subscribe(ChannelID::CMD_IN, &callback);
  lcm.subscribe(ChannelID::LIVE_IN, &LiveCallback); // 3 is incoming for live, 4 is out
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
      Run_Trajectory_State();
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
