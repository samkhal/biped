#include "Joint.h"

//Constructors
Joint::Joint() {};
Joint::Joint(int jointNumber_, int potPin_, int motorPin_, int enablePin_, float kP_, float kI_, float kD_, int direction_) :
  jointNumber(jointNumber_),
  potPin(potPin_),
  motorPin(motorPin_),
  enablePin(enablePin_),
  kP(kP_),
  kI(kI_),
  kD(kD_),
  direction(direction_){}

//Setters
void Joint::setMinPot(int minPot_){minPot = minPot_;}
void Joint::setMaxPot(int maxPot_){maxPot = maxPot_;}
void Joint::setMinTheta(int minTheta_){minTheta = minTheta_;} // in rads values
void Joint::setMaxTheta(int maxTheta_){maxTheta = maxTheta_;} // in rads values
void Joint::setZeroPot(int zeroPot_){zeroPot = zeroPot_;}
void Joint::setSetPoint(int setPoint_){setPoint = setPoint_;}
void Joint::setLocalJointNum(){localJointNum = ((jointNumber-1)%numOfJoints);}//get number from 0 to 2 instead of 1-12
void Joint::setMemoryAddr(JointROM memoryAddr_){memoryAddr = memoryAddr_;}

//Getters
int Joint::getJointNumber() {return jointNumber;}
int Joint::getMotorPin() {return motorPin;}
int Joint::getEnablePin() {return enablePin;}
int Joint::getMinPot() {return minPot;}
int Joint::getMaxPot() {return maxPot;}
int Joint::getMinTheta() {return minTheta;}
int Joint::getMaxTheta() {return maxTheta;}
int Joint::getZeroPot() {return zeroPot;}
int Joint::getLocalJointNum() {return localJointNum;}
JointROM Joint::getMemoryAddr(){return memoryAddr;}

int Joint::readROM(int address){
  uint16_t val;
  EEPROM.get(address,val);
  return val;
}

void Joint::writeROM_potRange(){
  EEPROM.put(memoryAddr.minPotAddr, (uint16_t)minPot);
  EEPROM.put(memoryAddr.maxPotAddr, (uint16_t)maxPot);
}

void Joint::writeROM_zeroTheta(){
  int val = readPotentiometer();
  EEPROM.put(memoryAddr.zeroPotAddr, (uint16_t)(val));
}

void Joint::writeROM_orientation(){
  EEPROM.put(memoryAddr.orientationAddr, (uint16_t)(direction));
}

//Other methods
int Joint::readPotentiometer(){return analogRead(potPin);}
void Joint::motorPWM(int drive){analogWrite(motorPin, drive);}
void Joint::setEnable(bool state){digitalWrite(enablePin, state);}
void Joint::setSetPointFromPot(){setPoint = readPotentiometer();}

void Joint::PIDcontrol() {
  int Actual = readPotentiometer();
  int Error = setPoint - Actual;
  float P = Error * kP; // calc proportional term
  // float D = ((joint->lastPID - Actual) * joint->getkD()) / PIDPeriod; // derivative term
  int Drive = P ;//+ D; // Total drive = P+I+D
  int sign = 1;
  if(direction == 0){// Check which direction to go.
    sign = -1;
  }
  Drive = (sign * Drive * ScaleFactor + (minPWM + maxPWM) / 2); // scale Drive to have 0 at 127
  if (Drive < minPWM) {
    Drive = minPWM;
  }
  if (Drive > maxPWM) {
    Drive = maxPWM;
  }
  motorPWM(Drive); // send PWM command to motor board
  lastPID = Actual;
}

bool Joint::checkOOR() {
  int pose = readPotentiometer();
  if (pose < ((int)readROM(memoryAddr.minPotAddr) + OutOfRangeThreshold) ||
   pose > ((int)readROM(memoryAddr.maxPotAddr) - OutOfRangeThreshold)) {
    return true;
  }
  return false;
}

bool Joint::CalibrationCheck() {
  int potVal = readPotentiometer();
  if (minPot > (uint16_t)potVal) {
    setMinPot((uint16_t)potVal);
  }
  else if (maxPot < (uint16_t)potVal) {
    setMaxPot((uint16_t)potVal);
  }
  if (potVal < minPotNaturalRange || potVal > maxPotNaturalRange) {
    return false;
  }
  return true;
}
