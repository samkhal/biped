  #include "WProgram.h"
  #include <EEPROM.h> //ROM memory library
  #include <TimerThree.h> // Timer library
  #include "slave_bridge.hpp" //LCM slave file
  #include "commData2Teensy.hpp" // header file for data from dispatcher to teensy
  #include "commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
  #include "error_channel.hpp"
  #include "Joint.hpp" // struct that stores joint data
  // #include "ConstJoint.h" // struct for constant joint data
  #include "ROM_DATA.h" // array to data addresses and enum

  LCMSerialSlave lcm; //initialize LCM object
  Joint joints[numOfJoints];

  // Channels
  const int IN  = 0;
  const int OUT = 1;
  const int ERROR = 2;

  // PID Variables
  const int PIDPeriod = 10; // milliseconds
  const int zeroTorque = 127; //PWM value for zero torque
  const int torqueOffset = 102; //max offset from zero torque
  const int minPWM = zeroTorque-torqueOffset; //27
  const int maxPWM = zeroTorque+torqueOffset; //229
  const int ScaleFactor = 1; // in case we need it
  const int IntThresh = 1;
  const int minPotNaturalRange = 10;
  const int maxPotNaturalRange = 1014;
  const int ticksPerRad = 172;// around 3 ticks per degree
  const int radsMultiplier = 1000;//positions in rads are multiplied by 1000
  const int OutOfRangeThreshold = 4;


  uint16_t minPot;
  uint16_t maxPot;

  //State Machine Variables
  enum COMMANDS{
    ID_REQUEST,
		START_CALIBRATION,
    STOP_CALIBRATION,
    GET_CALIBRATION,
		SEND_TRAJECTORY,
		RUN_STATIC_CONTROL,
		RUN_STATIC_ALL,
		RUN_TRAJECTORY,
		RUN_ALL_TRAJECTORIES,
    STOP,
		WAIT
  };
  int command = 0;
  bool EMERGENCY = false;
  int state = commData2Teensy::WAIT;
  int currJoint = 0;
  int currLocalJoint = 0;
  uint32_t dataLength;
  bool OutOfRange;
  volatile unsigned long int timer = 0;
  volatile bool allowPD = true; //flag for allowing the control

  //takes number of link/joint of teensy and an attribute
  // attributes: 0 - zeroTheta, 1 - minPot, 2 - maxPot, 3 - orientation
  int readROM_minPot(Joint joint) {
    int address1 = jointMem[joint.getLocalJointNum()].minPotAddr;
    int address2 = jointMem[joint.getLocalJointNum()].minPotAddr +1;
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }
  int readROM_maxPot(Joint joint) {
    int address1 = jointMem[joint.getLocalJointNum()].maxPotAddr;
    int address2 = jointMem[joint.getLocalJointNum()].maxPotAddr +1;
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }
  int readROM_zeroPot(Joint joint) {
    int address1 = jointMem[joint.getLocalJointNum()].zeroPotAddr;
    int address2 = jointMem[joint.getLocalJointNum()].zeroPotAddr +1;
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }
  int readROM_orientation(Joint joint) {
    int address1 = jointMem[joint.getLocalJointNum()].orientationAddr;
    int address2 = jointMem[joint.getLocalJointNum()].orientationAddr +1;
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }

  //Set the potentiometer max and min values
  void setPotRange(Joint* joint, int min, int max){
    EEPROM.write(jointMem[joint->getLocalJointNum()].minPotAddr, (byte)(joint->getMinPot()));
    EEPROM.write(jointMem[joint->getLocalJointNum()].minPotAddr +1, (byte)(joint->getMinPot() >> 8));
    EEPROM.write(jointMem[joint->getLocalJointNum()].maxPotAddr, (byte)(joint->getMaxPot()));
    EEPROM.write(jointMem[joint->getLocalJointNum()].maxPotAddr +1, (byte)(joint->getMaxPot() >> 8));
  }

  //Stores the orientation of motion in ROM
  void setOrientROM(Joint* joint){
    EEPROM.write(jointMem[joint->getLocalJointNum()].orientationAddr, (byte)(joint->getDirection()));
  }

  //Reads current angle and stores it as zero theta pot position in ROM
  void setZeroThetaROM(Joint* joint){
    int val = analogRead(joint->getPotPin());
    EEPROM.write(jointMem[joint->getLocalJointNum()].zeroPotAddr, (byte)(val));
    EEPROM.write(jointMem[joint->getLocalJointNum()].zeroPotAddr +1, (byte)(val >> 8));
  }

  //Checks if a joint goes very close to the min/max values and it stops it
  bool checkOOR(Joint joint) {
    int pose = analogRead(joint.getPotPin());
    if (pose < ((int)readROM_minPot(joint) + OutOfRangeThreshold) ||
     pose > ((int)readROM_maxPot(joint) - OutOfRangeThreshold)) {
      return true;
    }
    return false;
  }

  // //Takes a pointer of a Joint struct and it changes its data from rads to pot values
  // int radsToPot(int16_t data, ConstJoint cjoint){
  //   return ((data*ticksPerRad)/radsMultiplier + cjoint.zeroTheta);
  // }
  //
  // float potToRads(int val, ConstJoint cjoint){
  //   float out;
  //   val = val-cjoint.zeroTheta;
  //   out = (val/ticksPerRad)*radsMultiplier;
  //   return out;
  // }

  // Writes the the maximum and minimum pot values to ROM,
  // takes a pointer of constJoint struct, returns false if out of range
  bool CalibrationCheck(Joint* joint) {
    int potVal = analogRead(joint->getPotPin());
    if (minPot > (uint16_t)potVal) {
      minPot = (uint16_t)potVal;
    }
    else if (maxPot < (uint16_t)potVal) {
      maxPot = (uint16_t)potVal;
    }
    joint->setMinPot(minPot);
    joint->setMaxPot(maxPot);
    if (potVal < minPotNaturalRange || potVal > maxPotNaturalRange) {
      return false;
    }
    return true;
  }

  //PID control, takes a setpoint in pot values, a joint struct pointer and a constjoint struct (of the same joint)
  void PIDcontrol(int setPoint, Joint* joint) {
    int Actual = analogRead(joint->getPotPin());
    int Error = setPoint - Actual;
    float P = Error * joint->getkP(); // calc proportional term
    float D = ((joint->lastPID - Actual) * joint->getkD()) / PIDPeriod; // derivative term
    int Drive = P ;//+ D; // Total drive = P+I+D
    int sign = 1;
    if(joint->getDirection() == 0){// Check which direction to go.
      sign = -1;
    }
    Drive = (sign * Drive * ScaleFactor + (minPWM + maxPWM) / 2); // scale Drive to have 0 at 127
    if (Drive < minPWM) {
      Drive = minPWM;
    }
    if (Drive > maxPWM) {
      Drive = maxPWM;
    }
    analogWrite (joint->getMotorPin(), Drive); // send PWM command to motor board
    joint->lastPID = Actual;
  }

  //State machine functions
  void ID_Request(){
    commDataFromTeensy msgOut;
    int32_t links[numOfJoints];
    for (int i = 0; i<numOfJoints; i++){
      links[i] = EEPROM.read(jointMem[i].jointAddr);
      msgOut.joints[i] = links[i];
    }
    lcm.publish(OUT, &msgOut);
  }

  void Calibration_State(){
    if (CalibrationCheck(&joints[currJoint]) == false) {
      error_channel error_msg;
      error_msg.potHardwareOutOfRange = true;
      lcm.publish(ERROR, &error_msg);
    }
  }

  void Static_Control_State(){
    OutOfRange = checkOOR(joints[currLocalJoint]);
    if (OutOfRange) {
      error_channel error_msg;
      error_msg.isOutOfRange = true;
      lcm.publish(ERROR, &error_msg);
      command = COMMANDS::STOP;
      state = commData2Teensy::WAIT;
      EMERGENCY = true;
    }
    else{
      PIDcontrol((&joints[currLocalJoint])->setPoint, &joints[currLocalJoint]);
    }
  }

//Convert from Joint number 1-12 to 1-3 for local uses
  int convertToLocalJointNumber(int num){
    return ((num-1)%3)+1;
  }

  void Static_Control_All_State(){
    OutOfRange = checkOOR(joints[0])||checkOOR(joints[1])||checkOOR(joints[2]);
    if (OutOfRange) {
      error_channel error_msg;
      error_msg.isOutOfRange = true;
      lcm.publish(ERROR, &error_msg);
      command = COMMANDS::STOP;
      state = commData2Teensy::WAIT;
      EMERGENCY = true;
    }
    else{
      for (int i=0; i<3; i++){
        PIDcontrol((&joints[i])->setPoint, &joints[i]);
      }
    }
  }

  //Timer interrupt callback, increases the timer variable and sets the flag ready for PID control
  void timerCallback() {
    timer++;
    if (timer >= PIDPeriod) { //every 10 ms
      allowPD = true;
      timer = 0;
    }
  }
//================================STATE_ASSIGNMENT=======================================
  //State assignment
void stateAssignment(){
    switch (command) {

      case COMMANDS::ID_REQUEST:
        if (state == commData2Teensy::WAIT){
          ID_Request();
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case COMMANDS::START_CALIBRATION:
        if (state == commData2Teensy::WAIT){
          uint16_t val = (uint16_t)analogRead(joints[currLocalJoint].getPotPin());
          minPot = val;
          maxPot = val;
          (&joints[currLocalJoint])->setMinPot(val);
          (&joints[currLocalJoint])->setMaxPot(val);
          (&joints[currLocalJoint])->setZeroPot(val);
          state = commData2Teensy::CALIBRATION;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case COMMANDS::STOP_CALIBRATION:
        if (state == commData2Teensy::CALIBRATION){
          commDataFromTeensy msg_Out;
          msg_Out.minPot = (&joints[currLocalJoint])->getMinPot();
          msg_Out.maxPot = (&joints[currLocalJoint])->getMaxPot();
          setPotRange(&joints[currLocalJoint], (&joints[currLocalJoint])->getMinPot(), (&joints[currJoint])->getMaxPot());
          lcm.publish(OUT, &msg_Out);
          state = commData2Teensy::WAIT;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case COMMANDS::GET_CALIBRATION:
        break;

      case COMMANDS::SEND_TRAJECTORY:
        break;

      case COMMANDS::RUN_STATIC_CONTROL:
        if (state == commData2Teensy::WAIT){
          (&joints[currLocalJoint])->setPoint = analogRead((joints[currLocalJoint]).getPotPin());
          digitalWrite(joints[currLocalJoint].getEnablePin(), HIGH);
          state = commData2Teensy::RUN_STATIC_CONTROL;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case COMMANDS::RUN_STATIC_ALL:
        if (state == commData2Teensy::WAIT){
          for (int i=0; i<3; i++){
            (&joints[i])->setPoint = analogRead((&joints[i])->getPotPin());
            digitalWrite(joints[i].getEnablePin(), HIGH);
          }
          state = commData2Teensy::RUN_STATIC_ALL;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case COMMANDS::RUN_TRAJECTORY:
        break;

      case COMMANDS::RUN_ALL_TRAJECTORIES:
        break;

      case COMMANDS::STOP:
        for (int i=0; i<3; i++){
          digitalWrite(joints[i].getEnablePin(), LOW);
        }
        state = commData2Teensy::WAIT;
        EMERGENCY = false;
        break;

      case COMMANDS::WAIT:
        break;
    }
  }

//============================CALLBACK====================================
  void callback(CHANNEL_ID id, commData2Teensy* msg_IN){
    command = msg_IN->command;
    currJoint = msg_IN->joint;
    currLocalJoint = convertToLocalJointNumber(currJoint);
    dataLength = msg_IN->dataLength;
    stateAssignment();
  }


  //============================ MAIN LOOP ================================

  // Setup - Runs once
  void setup() {
    Serial.begin(115200);
    ROM_allocate(numOfJoints);
    for (int i=0; i<3; i++){
      joints[i] = JointTable[EEPROM.read(jointMem[i].jointAddr)];
      joints[i].setPoint = analogRead(joints[i].getPotPin());
      pinMode(joints[i].getMotorPin(), OUTPUT);
      pinMode(joints[i].getEnablePin(), OUTPUT);
      analogWrite(joints[i].getMotorPin(), zeroTorque);
      digitalWrite(joints[i].getEnablePin(), LOW);
      joints[i].setLocalJointNum();
      setOrientROM(&joints[i]);
      setZeroThetaROM(&joints[i]);
    }
    Timer3.initialize(1000); //1 ms
    Timer3.attachInterrupt(timerCallback);
    lcm.subscribe(IN, &callback); // 0 is incoming, 1 is out
  }

  // Main loop
  void loop() {
    if (EMERGENCY){
      stateAssignment();
    }
    lcm.handle();
    commDataFromTeensy msgOut;
    switch (state) {

      case commData2Teensy::CALIBRATION:
        Calibration_State();
        break;

      case commData2Teensy::RUN_STATIC_CONTROL :
        Static_Control_State();
        break;

      case commData2Teensy::RUN_STATIC_ALL :
        Static_Control_All_State();
        break;

      case commData2Teensy::RUN_TRAJECTORY :
      break;

      case commData2Teensy::RUN_ALL_TRAJECTORIES :
      break;

      case commData2Teensy::WAIT :
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
