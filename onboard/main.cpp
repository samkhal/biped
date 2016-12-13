  #include "WProgram.h"
  #include <TimerThree.h> // Timer library
  #include "slave_bridge.hpp" //LCM slave file
  #include "biped_lcm/commData2Teensy.hpp" // header file for data from dispatcher to teensy
  #include "biped_lcm/commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
  #include "biped_lcm/error_channel.hpp"
  #include "Joint.hpp" // struct that stores joint data
  #include <vector>

  using namespace biped_lcm;

  LCMSerialSlave lcm; //initialize LCM object
  std::vector<Joint> joints; //vector of joints

  // Channels
  const int IN  = 0;
  const int OUT = 1;
  const int ERROR = 2;

  // PID Variables
  const int PIDPeriod = 10; // milliseconds

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

  void stopMotors(){
    for (int i=0; i<numOfJoints; i++){
      joints[i].motorPWM(zeroTorque);
      joints[i].setEnable(LOW);
    }
  }

  //State machine functions
  void ID_Request(){
    commDataFromTeensy msgOut;
    int32_t links[numOfJoints];
    for (int i = 0; i<numOfJoints; i++){
      links[i] = EEPROM.read(joints[i].getMemoryAddr().jointAddr);
      msgOut.joints[i] = links[i];
    }
    lcm.publish(OUT, &msgOut);
  }

  void Calibration_State(){
    if ((&joints[currLocalJoint])->CalibrationCheck() == false) {
      error_channel error_msg;
      error_msg.potHardwareOutOfRange = true;
      lcm.publish(ERROR, &error_msg);
    }
  }

  void Static_Control_State(){
    bool OutOfRange = joints[currLocalJoint].checkOOR();
    if (OutOfRange) {
      error_channel error_msg;
      error_msg.isOutOfRange = true;
      lcm.publish(ERROR, &error_msg);
      state = STATES::WAIT;
    }
    else{
      (&joints[currLocalJoint])->PIDcontrol();
    }
  }

//Convert from Joint number 1-12 to 0-2 for local uses
  int convertToLocalJointNumber(int num){
    return ((num-1)%numOfJoints);
  }

  void Static_Control_All_State(){
    bool OutOfRange = joints[0].checkOOR()||joints[1].checkOOR()||joints[2].checkOOR();
    if (OutOfRange) {
      error_channel error_msg;
      error_msg.isOutOfRange = true;
      lcm.publish(ERROR, &error_msg);
      state = STATES::WAIT;
    }
    else{
      for (int i=0; i<numOfJoints; i++){
        (&joints[i])->PIDcontrol();
      }
    }
  }

  //Timer interrupt callback, increases the timer variable and sets the flag ready for PID control
  void timerCallback() {
    timer++;
    if (timer >= PIDPeriod) { //every 10 ms
      timer = 0;
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
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case commData2Teensy::START_CALIBRATION:
        if (state == STATES::WAIT){
          uint16_t val = (uint16_t)joints[currLocalJoint].readPotentiometer();
          (&joints[currLocalJoint])->setMinPot(val);
          (&joints[currLocalJoint])->setMaxPot(val);
          (&joints[currLocalJoint])->setZeroPot(val);
          state = STATES::CALIBRATION;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case commData2Teensy::STOP_CALIBRATION:
        if (state == STATES::CALIBRATION){
          commDataFromTeensy msg_Out;
          msg_Out.minPot = (&joints[currLocalJoint])->getMinPot();
          msg_Out.maxPot = (&joints[currLocalJoint])->getMaxPot();
          (&joints[currLocalJoint])->writeROM_potRange();
          lcm.publish(OUT, &msg_Out);
          state = STATES::WAIT;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case commData2Teensy::GET_CALIBRATION:
        break;

      case commData2Teensy::RECEIVE_TRAJECTORY:
        break;

      case commData2Teensy::RUN_STATIC_CONTROL:
        if (state == STATES::WAIT){
          (&joints[currLocalJoint])->setSetPointFromPot();
          joints[currLocalJoint].setEnable(HIGH);
          state = commData2Teensy::RUN_STATIC_CONTROL;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case commData2Teensy::RUN_STATIC_ALL:
        if (state == STATES::WAIT){
          for (int i=0; i<3; i++){
            (&joints[i])->setSetPointFromPot();
            joints[i].setEnable(HIGH);
          }
          state = commData2Teensy::RUN_STATIC_ALL;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
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
    ROM_allocate(numOfJoints);
    for (int i=0; i<numOfJoints; i++){
      uint16_t index;
      EEPROM.get(jointMem[i].jointAddr,index);
      joints.push_back(JointTable[index-1]);//because joint 1 is JointTable[0]
      joints[i].setSetPointFromPot();
      joints[i].setMemoryAddr(jointMem[i]);
      pinMode(joints[i].getMotorPin(), OUTPUT);
      pinMode(joints[i].getEnablePin(), OUTPUT);
      joints[i].motorPWM(zeroTorque);
      joints[i].setEnable(LOW);
      joints[i].setLocalJointNum();
      (&joints[i])->writeROM_orientation();
      (&joints[i])->writeROM_zeroTheta();
    }
    Timer3.initialize(1000); //1 ms
    Timer3.attachInterrupt(timerCallback);
    lcm.subscribe(IN, &callback); // 0 is incoming, 1 is out
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
