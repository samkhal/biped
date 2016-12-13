  #include "WProgram.h"
  #include <EEPROM.h> //ROM memory library
  #include <TimerThree.h> // Timer library
  #include "slave_bridge.hpp" //LCM slave file
  #include "commData2Teensy.hpp" // header file for data from dispatcher to teensy
  #include "commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
  #include "error_channel.hpp"
  #include "Joint.hpp" // struct that stores joint data

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
  enum STATES{
    CALIBRATION,
    SEND_TRAJECTORY,
    RUN_STATIC_CONTROL,
    RUN_STATIC_ALL,
    RUN_TRAJECTORY,
    RUN_ALL_TRAJECTORIES,
    WAIT
  };

  int command = 0;
  bool EMERGENCY = false;
  int state = commData2Teensy::WAIT;
  int currLocalJoint = 0;
  uint32_t dataLength;
  bool OutOfRange;
  volatile unsigned long int timer = 0;
  volatile bool allowPD = true; //flag for allowing the control

  int readROM_minPot(Joint joint) {
    int address1 = joint.getMemoryAddr().minPotAddr;
    int address2 = joint.getMemoryAddr().minPotAddr +1;
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }
  int readROM_maxPot(Joint joint) {
    int address1 = joint.getMemoryAddr().maxPotAddr;
    int address2 = joint.getMemoryAddr().maxPotAddr +1;
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }
  int readROM_zeroPot(Joint joint) {
    int address1 = joint.getMemoryAddr().zeroPotAddr;
    int address2 = joint.getMemoryAddr().zeroPotAddr +1;
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }
  int readROM_orientation(Joint joint) {
    int address1 = joint.getMemoryAddr().orientationAddr;
    int address2 = joint.getMemoryAddr().orientationAddr +1;
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }

  //Set the potentiometer max and min values
  void setPotRange(Joint* joint, int min, int max){
    EEPROM.write(joint->getMemoryAddr().minPotAddr, (byte)min);
    EEPROM.write(joint->getMemoryAddr().minPotAddr +1, (byte)(min >> 8));
    EEPROM.write(joint->getMemoryAddr().maxPotAddr, (byte)max);
    EEPROM.write(joint->getMemoryAddr().maxPotAddr +1, (byte)(max >> 8));
  }

  //Stores the orientation of motion in ROM
  void setOrientROM(Joint* joint){
    EEPROM.write(joint->getMemoryAddr().orientationAddr, (byte)(joint->getDirection()));
  }

  //Reads current angle and stores it as zero theta pot position in ROM
  void setZeroThetaROM(Joint* joint){
    int val = joint->readPotentiometer();
    EEPROM.write(joint->getMemoryAddr().zeroPotAddr, (byte)(val));
    EEPROM.write(joint->getMemoryAddr().zeroPotAddr +1, (byte)(val >> 8));
  }

  //Checks if a joint goes very close to the min/max values and it stops it
  bool checkOOR(Joint joint) {
    int pose = joint.readPotentiometer();
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
    int potVal = joint->readPotentiometer();
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
    int Actual = joint->readPotentiometer();
    int Error = setPoint - Actual;
    float P = Error * joint->getkP(); // calc proportional term
    // float D = ((joint->lastPID - Actual) * joint->getkD()) / PIDPeriod; // derivative term
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
    joint->motorPWM(Drive); // send PWM command to motor board
    joint->lastPID = Actual;
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
    if (CalibrationCheck(&joints[currLocalJoint]) == false) {
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
      command = commData2Teensy::STOP;
      state = STATES::WAIT;
      EMERGENCY = true;
    }
    else{
      PIDcontrol((&joints[currLocalJoint])->setPoint, &joints[currLocalJoint]);
    }
  }

//Convert from Joint number 1-12 to 0-2 for local uses
  int convertToLocalJointNumber(int num){
    return ((num-1)%numOfJoints);
  }

  void Static_Control_All_State(){
    OutOfRange = checkOOR(joints[0])||checkOOR(joints[1])||checkOOR(joints[2]);
    if (OutOfRange) {
      error_channel error_msg;
      error_msg.isOutOfRange = true;
      lcm.publish(ERROR, &error_msg);
      command = commData2Teensy::STOP;
      state = STATES::WAIT;
      EMERGENCY = true;
    }
    else{
      for (int i=0; i<numOfJoints; i++){
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
          minPot = val;
          maxPot = val;
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
          setPotRange(&joints[currLocalJoint], (&joints[currLocalJoint])->getMinPot(), (&joints[currLocalJoint])->getMaxPot());
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

      case commData2Teensy::SEND_TRAJECTORY:
        break;

      case commData2Teensy::RUN_STATIC_CONTROL:
        if (state == commData2Teensy::WAIT){
          (&joints[currLocalJoint])->setSetPointFromPot();
          digitalWrite(joints[currLocalJoint].getEnablePin(), HIGH);
          state = commData2Teensy::RUN_STATIC_CONTROL;
        }
        else{
          error_channel msg_Out;
          msg_Out.inappropriateCommand = true;
          lcm.publish(OUT, &msg_Out);
        }
        break;

      case commData2Teensy::RUN_STATIC_ALL:
        if (state == commData2Teensy::WAIT){
          for (int i=0; i<3; i++){
            (&joints[i])->setSetPointFromPot();
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

      case commData2Teensy::RUN_TRAJECTORY:
        break;

      case commData2Teensy::RUN_ALL_TRAJECTORIES:
        break;

      case commData2Teensy::STOP:
        for (int i=0; i<3; i++){
          digitalWrite(joints[i].getEnablePin(), LOW);
        }
        state = commData2Teensy::WAIT;
        EMERGENCY = false;
        break;

      case commData2Teensy::WAIT:
        state = commData2Teensy::WAIT;
        break;
    }
  }

//============================CALLBACK====================================
  void callback(CHANNEL_ID id, commData2Teensy* msg_IN){
    command = msg_IN->command;
    int currJoint = msg_IN->joint;
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
      joints[i].setSetPointFromPot();
      joints[i].setMemoryAddr(jointMem[i]);
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
