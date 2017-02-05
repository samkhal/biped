#ifndef JOINT_H_
#define JOINT_H_
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
    Joint();
    Joint(int jointNumber_, int potPin_, int motorPin_, int enablePin_, float kP_, float kI_, float kD_, int direction_);

    //Setters
    void setMinPot(int minPot_);
    void setMaxPot(int maxPot_);
    void setMinTheta(int minTheta_); // in rads values
    void setMaxTheta(int maxTheta_); // in rads values
    void setZeroPot(int zeroPot_);
    void setSetPoint(int setPoint_);
    void setLocalJointNum();//get number from 0 to 2 instead of 1-12
    void setMemoryAddr(JointROM memoryAddr_);

    //Getters
    int getJointNumber();
    int getMotorPin();
    int getEnablePin();
    int getMinPot();
    int getMaxPot();
    int getMinTheta();
    int getMaxTheta();
    int getZeroPot();
    int getLocalJointNum();
    JointROM getMemoryAddr();

    //Method to read data from ROM
    int readROM(int address);

    //Methods to write data to ROM
    void writeROM_potRange();
    void writeROM_zeroTheta();
    void writeROM_orientation();

    //Other methods
    int readPotentiometer();
    void motorPWM(int drive);
    void setEnable(bool state);
    void setSetPointFromPot();
    //PID control, takes a setpoint in pot values, a joint struct pointer and a constjoint struct (of the same joint)
    void PIDcontrol();
    //Checks if a joint goes very close to the min/max values and it stops it
    bool checkOOR();
    //Writes the the maximum and minimum pot values, checks for physical OutOfRange
    bool CalibrationCheck();

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
    const int numOfJoints = 3;
    JointROM memoryAddr;
};

#endif
