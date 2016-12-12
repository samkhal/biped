  #include "WProgram.h"
  #include <EEPROM.h> //ROM memory library
  #include <TimerThree.h> // Timer library
  #include "slave_bridge.hpp" //LCM slave file
  #include "commData2Teensy.hpp" // header file for data from dispatcher to teensy
  #include "commDataFromTeensy.hpp" // header file for data from teensy to dispatcher
  #include "error_channel.hpp"
  #include "Joint.h" // struct that stores joint data
  #include "ConstJoint.h" // struct for constant joint data
  #include "ROM_DATA.h" // array to data addresses and enum

  LCMSerialSlave lcm; //initialize LCM object

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

  //^probably gonna change with LCM
  uint16_t minPot;
  uint16_t maxPot;

  //State Machine Variables
  int action = commData2Teensy::WAIT;
  int link = 1;
  uint32_t dataLength;
  bool calibrationFlag = true;
  bool staticControlFlag = true;
  bool OutOfRange;
  volatile unsigned long int timer = 0;
  volatile bool allowPD = true; //flag for allowing the control

  //Array of constant joint params
  ConstJoint linksConst[3] = {
    JointTable[EEPROM.read(ROM_ENUM::link1Addr)],
    JointTable[EEPROM.read(ROM_ENUM::link2Addr)],
    JointTable[EEPROM.read(ROM_ENUM::link3Addr)]};

  //Array of joints initialization
  Joint links [3];

  //Temp joints
  Joint tempJoint;
  ConstJoint tempCJoint;

  //takes number of link/joint of teensy and an attribute
  // attributes: 0 - zeroTheta, 1 - minPot, 2 - maxPot, 3 - orientation
  int readROM(int link, int attribute) {
    int address1 = ROM[(link*2-1) + attribute*6 + 2];
    int address2 = ROM[(link*2) + attribute*6 + 2];
    int val = EEPROM.read(address1);
    val = val + (EEPROM.read(address2) << 8);
    return val;
  }

  //Set the potentiometer max and min values
  void setPotRange(int joint, int min, int max){
    ConstJoint* cjoint = &tempCJoint;
    EEPROM.write(ROM_MIN_POT[joint * MEM_BYTES], (byte)(cjoint->minPot));
    EEPROM.write(ROM_MIN_POT[joint * MEM_BYTES +1], (byte)(cjoint->minPot >> 8));
    EEPROM.write(ROM_MAX_POT[joint * MEM_BYTES], (byte)(cjoint->minPot));
    EEPROM.write(ROM_MAX_POT[joint * MEM_BYTES +1], (byte)(cjoint->minPot >> 8));
  }

  //Stores the orientation of motion in ROM
  void setOrientROM(ConstJoint* cjoint){
    EEPROM.write(ROM_ORIENTATION[cjoint->link * MEM_BYTES], (byte)(cjoint->direction));
  }

  //Reads current angle and stores ia as zero theta pot position in ROM
  void setZeroThetaROM(ConstJoint* cjoint){
    int val = analogRead(cjoint->position);
    EEPROM.write(ROM_ZERO_THETA[cjoint->link * MEM_BYTES], (byte)(val));
    EEPROM.write(ROM_ZERO_THETA[cjoint->link * MEM_BYTES +1], (byte)(val >> 8));
  }

  //Checks if a joint goes very close to the min/max values and it stops it
  bool checkOOR(ConstJoint cjoint) {
    int pose = analogRead(cjoint.position);
    int i = cjoint.link;
    if (pose < ((int)readROM(i, ROM_ATTR::MIN_POT) + OutOfRangeThreshold) ||
     pose > ((int)readROM(i, ROM_ATTR::MIN_POT) - OutOfRangeThreshold)) {
      return true;
    }
    return false;
  }

  //Takes a pointer of a Joint struct and it changes its data from rads to pot values
  int radsToPot(int16_t data, ConstJoint cjoint){
    return ((data*ticksPerRad)/radsMultiplier + cjoint.zeroTheta);
  }

  float potToRads(int val, ConstJoint cjoint){
    float out;
    val = val-cjoint.zeroTheta;
    out = (val/ticksPerRad)*radsMultiplier;
    return out;
  }

  //Writes the the maximum and minimum pot values to ROM,
  //takes a pointer of constJoint struct, returns false if out of range
  bool Calibration(ConstJoint* tempCJoint) {
    int potVal = analogRead(tempCJoint->position);
    if (minPot > (uint16_t)potVal) {
      minPot = (uint16_t)potVal;
    }
    else if (maxPot < (uint16_t)potVal) {
      maxPot = (uint16_t)potVal;
    }
    tempCJoint->minPot = minPot;
    tempCJoint->maxPot = maxPot;
    if (potVal < minPotNaturalRange || potVal > maxPotNaturalRange) {
      return false;
    }
    return true;
  }

  //PID control, takes a setpoint in pot values, a joint struct pointer and a constjoint struct (of the same joint)
  void PIDcontrol(int setPoint, Joint* tempJoint, ConstJoint tempCJoint) {
    int Actual = analogRead(tempCJoint.position);
    int Error = setPoint - Actual;
    float P = Error * tempCJoint.kP; // calc proportional term
    float D = ((tempJoint->lastPID - Actual) * tempCJoint.kD) / PIDPeriod; // derivative term
    int Drive = P ;//+ D; // Total drive = P+I+D
    int sign = 1;
    if(tempCJoint.direction == 0){// Check which direction to go.
      sign = -1;
    }
    Drive = (sign * Drive * ScaleFactor + (minPWM + maxPWM) / 2); // scale Drive to have 0 at 127
    if (Drive < minPWM) {
      Drive = minPWM;
    }
    if (Drive > maxPWM) {
      Drive = maxPWM;
    }
    analogWrite (tempCJoint.motor, Drive); // send PWM command to motor board
    tempJoint->lastPID = Actual;
  }

  //set the appropriate temp joints
  void jointSelection(int32_t link){
    tempJoint = links[link-1];
    tempCJoint = linksConst[link-1];
  }

  //Timer interrupt callback, increases the timer variable and sets the flag ready for PID control
  void timerCallback() {
    timer++;
    if (timer >= PIDPeriod) { //every 10 ms
      allowPD = true;
      timer = 0;
    }
  }

  //Callback from lcm messages
  void callback(CHANNEL_ID id, commData2Teensy* msg_IN){
    action = msg_IN->command;
    link = msg_IN->joint;
    dataLength = msg_IN->dataLength;
    commDataFromTeensy msg_OUT;
    msg_OUT.joint1 = msg_IN->command;
    lcm.publish(1, &msg_OUT);
  }


  //============================ MAIN LOOP ================================

  // Setup - Runs once
  void setup() {
    Serial.begin(115200);
    for (int i=0; i<3; i++){
      links[i].setPoint = analogRead(linksConst[i].position);
      pinMode(linksConst[i].motor, OUTPUT);
      pinMode(linksConst[i].enable, OUTPUT);
      analogWrite(linksConst[i].motor, zeroTorque);
      digitalWrite(linksConst[i].enable, LOW);
      setOrientROM(&linksConst[i]);
      setZeroThetaROM(&linksConst[i]);
    }
    Timer3.initialize(1000); //1 ms
    Timer3.attachInterrupt(timerCallback);
    lcm.subscribe(IN, &callback); // 0 is incoming, 1 is out
  }

  // Main loop
  void loop() {
    lcm.handle();
    jointSelection(link);
    commDataFromTeensy msgOut;
    switch (action) {

      case commData2Teensy::ID_REQUEST:
        int32_t link1;
        int32_t link2;
        int32_t link3;
        link1 = EEPROM.read(ROM_ENUM::link1Addr);
        link2 = EEPROM.read(ROM_ENUM::link2Addr);
        link3 = EEPROM.read(ROM_ENUM::link3Addr);
        msgOut.joint1 = link1;
        msgOut.joint2 = link2;
        msgOut.joint3 = link3;
        lcm.publish(OUT, &msgOut);
        break;

      case commData2Teensy::START_CALIBRATION :
        if (calibrationFlag==true){
          minPot = (uint16_t)analogRead(tempCJoint.position);
          maxPot = minPot;
          (&tempCJoint)->minPot = (uint16_t)tempJoint.setPoint;
          (&tempCJoint)->maxPot = (uint16_t)tempJoint.setPoint;
          (&tempCJoint)->zeroTheta = (uint16_t)analogRead(tempCJoint.position);
          calibrationFlag = false;
        }
        if (Calibration(&tempCJoint) == false) {
          error_channel error_msg;
          error_msg.potHardwareOutOfRange = true;
          lcm.publish(ERROR, &error_msg);
        }
        else {
          (&tempCJoint)->minPot = (uint16_t)tempJoint.setPoint;
          (&tempCJoint)->maxPot = (uint16_t)tempJoint.setPoint;
        }
        break;

      case commData2Teensy::STOP_CALIBRATION :
        msgOut.minPot = (&tempCJoint)->minPot;
        msgOut.maxPot = (&tempCJoint)->maxPot;
        setPotRange((&tempCJoint)->link, minPot, maxPot);
        lcm.publish(OUT, &msgOut);
        calibrationFlag = true;
        action =commData2Teensy::WAIT;
        break;

      case commData2Teensy::SEND_TRAJECTORY :
        break;

      case commData2Teensy::RUN_STATIC_CONTROL :
        if (staticControlFlag == true){
          (&tempJoint)->setPoint = analogRead((&tempCJoint)->position);
          digitalWrite(tempCJoint.enable, HIGH);
          staticControlFlag = false;
        }

        OutOfRange = checkOOR(tempCJoint);
        if (OutOfRange) {
          error_channel error_msg;
          error_msg.isOutOfRange = true;
          lcm.publish(ERROR, &error_msg);
          action = commData2Teensy::STOP;
        }
        else{
          PIDcontrol((&tempJoint)->setPoint, &tempJoint, tempCJoint);
        }
        break;

      case commData2Teensy::RUN_STATIC_ALL :
        if (staticControlFlag == true){
          for (int i=0; i<3; i++){
            (&links[i])->setPoint = analogRead((&linksConst[i])->position);
            digitalWrite(linksConst[i].enable, HIGH);
          }
          staticControlFlag = false;
        }
        OutOfRange = checkOOR(linksConst[0])||checkOOR(linksConst[1])||checkOOR(linksConst[2]);
        if (OutOfRange) {
          error_channel error_msg;
          error_msg.isOutOfRange = true;
          lcm.publish(ERROR, &error_msg);
          action = commData2Teensy::STOP;
        }
        else{
          for (int i=0; i<3; i++){
            PIDcontrol((&links[i])->setPoint, &links[i], linksConst[i]);
          }
        }
        break;

      case commData2Teensy::RUN_TRAJECTORY :
      break;

      case commData2Teensy::RUN_ALL_TRAJECTORIES :
      break;

      case commData2Teensy::STOP :
        for (int i=0; i<3; i++){
          digitalWrite(linksConst[i].enable, LOW);
        }
        staticControlFlag = true;
        action = commData2Teensy::WAIT;
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
