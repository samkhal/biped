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
const int minPotNaturalRange = 15; //+15
const int maxPotNaturalRange = 4081; //-15
const int PIDPeriod = 1;//ms
const float potTicksPerRad = 706.6; // for 12 bit resolution ADC

class Joint {
  public:
    //Constructors
    Joint();
    Joint(int jointNumber_, int potPin_, int motorPin_, int enablePin_, float kP_, float kI_, float kD_, int motorOrientation_, int potOrientation_, int potCabling_, bool outOfRangeFlag_);

    //Setters
    void setMinPot(int minPot_);
    void setMaxPot(int maxPot_);
    void setMinTheta(int minTheta_); // in rads values
    void setMaxTheta(int maxTheta_); // in rads values
    void setZeroPot(int zeroPot_);
    void setSetPoint(float angleSetPoint_);
    void setMemoryAddr(JointROM memoryAddr_);
    void setDirection();

    //Getters
    int getJointNumber();
    int getMotorPin();
    int getEnablePin();
    int getMinPot();
    int getMaxPot();
    int getMinTheta();
    int getMaxTheta();
    int getZeroPot();
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
    int PIDcontrol();
    //Checks if a joint goes very close to the min/max values and it stops it
    bool checkOOR();
    //Writes the the maximum and minimum pot values, checks for physical OutOfRange
    bool CalibrationCheck();
    //Converts angle (rads) to potentiometer ticks
    int convertRadsToTicks(float rads);
    float convertTicksToRads(int ticks);

  private:
    const int jointNumber;
    const int potPin;
    const int motorPin;
    const int enablePin;
    const float kP;
    const float kI;
    const float kD;
    const int motorOrientation;
    const int potOrientation;
    const int potCabling;
    const bool outOfRangeFlag;// flag to whether we check if this joint is out of range (it can cause problem with knee)
    int direction;
    int minPot;
    int maxPot;
    int minTheta; // in rads values
    int maxTheta; // in rads values
    int zeroPot; // in pot values
    int lastPID;
    float angleSetPoint;
    int potSetPoint;
    const int numOfJoints = 3;
    JointROM memoryAddr;
};

#endif
