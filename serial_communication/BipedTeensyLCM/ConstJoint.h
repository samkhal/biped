#ifndef CONSTJOINT_H_
#define CONSTJOINT_H_

struct ConstJoint {
  int link;
  int position;
  int motor;
  int enable;
  float kP;
  float kI;
  float kD;
  int direction;
  uint8_t channel;
  int motorOrientation;
  int potOrientation;
  int potCabling;
  char name[10];
  int lastPID;
  int minPot;
  int maxPot;
  int minTheta; // in rads values
  int maxTheta; // in rads values
  int zeroTheta; // in pot values
};

#endif
