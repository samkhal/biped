#include "Joint.h"

//Constructors
Joint::Joint() {};
Joint::Joint(int jointNumber_, int potPin_, int motorPin_, int enablePin_, float kP_, float kI_, float kD_, int motorOrientation_, int potOrientation_, int potCabling_, bool outOfRangeFlag_) :
  jointNumber(jointNumber_),
  potPin(potPin_),
  motorPin(motorPin_),
  enablePin(enablePin_),
  kP(kP_),
  kI(kI_),
  kD(kD_),
  motorOrientation(motorOrientation_),
  potOrientation(potOrientation_),
  potCabling(potCabling_),
  outOfRangeFlag(outOfRangeFlag_){}

//Setters
void Joint::setMinPot(int minPot_){minPot = minPot_;}
void Joint::setMaxPot(int maxPot_){maxPot = maxPot_;}
void Joint::setMinTheta(int minTheta_){minTheta = minTheta_;} // in rads values
void Joint::setMaxTheta(int maxTheta_){maxTheta = maxTheta_;} // in rads values
void Joint::setZeroPot(int zeroPot_){zeroPot = zeroPot_;}
void Joint::setSetPoint(float angleSetPoint_){potSetPoint = int(potTicksPerRad*angleSetPoint_) + zeroPot; angleSetPoint = angleSetPoint_;}
void Joint::setMemoryAddr(JointROM memoryAddr_){memoryAddr = memoryAddr_;}
void Joint::setDirection(){direction = motorOrientation*potOrientation*potCabling;}

//Getters
int Joint::getJointNumber() {return jointNumber;}
int Joint::getMotorPin() {return motorPin;}
int Joint::getEnablePin() {return enablePin;}
int Joint::getMinPot() {return minPot;}
int Joint::getMaxPot() {return maxPot;}
int Joint::getMinTheta() {return minTheta;}
int Joint::getMaxTheta() {return maxTheta;}
int Joint::getZeroPot() {return zeroPot;}
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
void Joint::motorPWM(int drive){analogWrite(motorPin,drive);}
void Joint::setEnable(bool state){digitalWrite(enablePin, state);}
void Joint::setSetPointFromPot(){potSetPoint = readPotentiometer();}

int Joint::PIDcontrol() {
  int Actual = readPotentiometer();
  int Error = potSetPoint - Actual;
  float P = Error * kP; // calc proportional term
  float D = ((lastPID - Actual) * kD) / PIDPeriod; // derivative term
  int Drive = P + D; // Total drive = P+I+D
  Drive = (direction * Drive * ScaleFactor + (minPWM + maxPWM) / 2); // scale Drive to have 0 at 127
  if (Drive < minPWM) {
    Drive = minPWM;
  }
  if (Drive > maxPWM) {
    Drive = maxPWM;
  }
  motorPWM(Drive); // send PWM command to motor board
  lastPID = Actual;
  return Drive;
}

bool Joint::checkOOR() {
  if (outOfRangeFlag){
    int pose = readPotentiometer();
    if (pose < ((int)readROM(memoryAddr.minPotAddr) + OutOfRangeThreshold) ||
     pose > ((int)readROM(memoryAddr.maxPotAddr) - OutOfRangeThreshold)) {
      return true;
    }
    return false;
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
