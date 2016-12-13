#ifndef JOINT_HPP_
#define JOINT_HPP_
#include "ROM_DATA.h" // array to data addresses and enum

class Joint {
  public:
    int lastPID;
    int setPoint;

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
    void setDirection(int direction_){direction = direction_;}
    void setMotorOrientation(int motorOrientation_){motorOrientation = motorOrientation_;}
    void setPotOrientation(int potOrientation_){potOrientation = potOrientation_;}
    void setPotCabling(int potCabling_) {potCabling = potCabling_;};
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
    void setSetPointFromPot(){setPoint = readPotentiometer();}

  private:
    int jointNumber;
    int potPin;
    int motorPin;
    int enablePin;
    float kP;
    float kI;
    float kD;
    int direction;
    int localJointNum;
    int motorOrientation;
    int potOrientation;
    int potCabling;
    int minPot;
    int maxPot;
    int minTheta; // in rads values
    int maxTheta; // in rads values
    int zeroPot; // in pot values
    JointROM memoryAddr;
};


Joint JointTable[12] = {
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
