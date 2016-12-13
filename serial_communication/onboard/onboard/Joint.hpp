#ifndef JOINT_HPP_
#define JOINT_HPP_
#include <EEPROM.h> //ROM memory library
#include "ROM_DATA.h" // array to data addresses and enum

const int zeroTorque = 127; //PWM value for zero torque
const int torqueOffset = 102; //max offset from zero torque
const int minPWM = zeroTorque-torqueOffset; //27
const int maxPWM = zeroTorque+torqueOffset; //229
const int ScaleFactor = 1; // in case we need it
const int OutOfRangeThreshold = 4;
const int minPotNaturalRange = 10;
const int maxPotNaturalRange = 1014;

class Joint {
  public:
    //Constructors
    Joint() {};
    Joint(int jointNumber_, int potPin_, int motorPin_, int enablePin_, float kP_, float kI_, float kD_, int direction_) :
      jointNumber(jointNumber_),
      potPin(potPin_),
      motorPin(motorPin_),
      enablePin(enablePin_),
      kP(kP_),
      kI(kI_),
      kD(kD_),
      direction(direction_) {}

    //Setters
    void setMinPot(int minPot_){minPot = minPot_;}
    void setMaxPot(int maxPot_){maxPot = maxPot_;}
    void setMinTheta(int minTheta_){minTheta = minTheta_;} // in rads values
    void setMaxTheta(int maxTheta_){maxTheta = maxTheta_;} // in rads values
    void setZeroPot(int zeroPot_){zeroPot = zeroPot_;}
    void setLocalJointNum(){localJointNum = ((jointNumber-1)%numOfJoints);}//get number from 0 to 2 instead of 1-12
    void setMemoryAddr(JointROM memoryAddr_){memoryAddr = memoryAddr_;}

    //Getters
    int getJointNumber() {return jointNumber;}
    int getPotPin() {return potPin;}
    int getMotorPin() {return motorPin;}
    int getEnablePin() {return enablePin;}
    float getkP () {return kP;}
    float getkI () {return kI;}
    float getkD () {return kD;}
    int getDirection () {return direction;}
    int getMotorOrientation() {return motorOrientation;}
    int getPotOrientation() {return potOrientation;}
    int getPotCabling() {return potCabling;}
    int getMinPot() {return minPot;}
    int getMaxPot() {return maxPot;}
    int getMinTheta() {return minTheta;}
    int getMaxTheta() {return maxTheta;}
    int getZeroPot() {return zeroPot;}
    int getLocalJointNum() {return localJointNum;}
    JointROM getMemoryAddr(){return memoryAddr;}

    //Other methods
    int readPotentiometer(){return analogRead(potPin);}
    void motorPWM(int drive){analogWrite(motorPin,drive);}
    void setEnable(bool state){digitalWrite(enablePin, state);}
    void setSetPointFromPot(){setPoint = readPotentiometer();}
    int readROM_joint(){
      uint16_t val;
      EEPROM.get(memoryAddr.jointAddr,val);
      return val;
    }
    int readROM_minPot(){
      uint16_t val;
      EEPROM.get(memoryAddr.minPotAddr,val);
      return val;
    }
    int readROM_maxPot(){
      uint16_t val;
      EEPROM.get(memoryAddr.maxPotAddr,val);
      return val;
    }
    int readROM_zeroPot(){
      uint16_t val;
      EEPROM.get(memoryAddr.zeroPotAddr,val);
      return val;
    }
    int readROM_orientation(){
      uint16_t val;
      EEPROM.get(memoryAddr.orientationAddr,val);
      return val;
    }
    void writeROM_potRange(){
      EEPROM.put(memoryAddr.minPotAddr, (uint16_t)minPot);
      EEPROM.put(memoryAddr.maxPotAddr, (uint16_t)maxPot);
    }
    void writeROM_zeroTheta(){
      int val = readPotentiometer();
      EEPROM.put(memoryAddr.zeroPotAddr, (uint16_t)(val));
    }
    void writeROM_orientation(){
      EEPROM.put(memoryAddr.orientationAddr, (uint16_t)(direction));
    }

    //PID control, takes a setpoint in pot values, a joint struct pointer and a constjoint struct (of the same joint)
    void PIDcontrol() {
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

    //Checks if a joint goes very close to the min/max values and it stops it
    bool checkOOR() {
      int pose = readPotentiometer();
      if (pose < ((int)readROM_minPot() + OutOfRangeThreshold) ||
       pose > ((int)readROM_maxPot() - OutOfRangeThreshold)) {
        return true;
      }
      return false;
    }

    // Writes the the maximum and minimum pot values to ROM,
    // takes a pointer of constJoint struct, returns false if out of range
    bool CalibrationCheck() {
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

  private:
    const int jointNumber;
    const int potPin;
    const int motorPin;
    const int enablePin;
    const float kP;
    const float kI;
    const float kD;
    const int direction;
    int localJointNum;
    const int motorOrientation;
    const int potOrientation;
    const int potCabling;
    int minPot;
    int maxPot;
    int minTheta; // in rads values
    int maxTheta; // in rads values
    int zeroPot; // in pot values
    int lastPID;
    int setPoint;
    JointROM memoryAddr;
};


const Joint JointTable[12] = {
//  Joint#, PotPin, MotorPin, EnablePin, kP,  kI,  kD,   Direction
Joint(1,   A9,       5,        0,      0.1f,   0,  0.1f,   1),
Joint(2,   A7,       4,        1,      0.1f,   0,  0.1f,   0),
Joint(3,   A8,       3,        2,      0.1f,   0,  0.1f,   1),
Joint(4,   A9,       5,        0,      0.1f,   0,  0.1f,   1),
Joint(5,   A7,       4,        1,      0.1f,   0,  0.1f,   0),
Joint(6,   A8,       3,        2,      0.1f,   0,  0.1f,   1),
Joint(7,   A9,       5,        0,      0.1f,   0,  0.1f,   1),
Joint(8,   A7,       4,        1,      0.1f,   0,  0.1f,   0),
Joint(9,   A8,       3,        2,      0.1f,   0,  0.1f,   1),
Joint(10,  A9,       5,        0,      0.1f,   0,  0.1f,   1),
Joint(11,  A7,       4,        1,      0.1f,   0,  0.1f,   0),
Joint(12,  A8,       3,        2,      0.1f,   0,  0.1f,   1)
};
#endif
