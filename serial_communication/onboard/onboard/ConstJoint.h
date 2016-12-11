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

const ConstJoint JointTable[13]= {
// L, P,  M, E,    kP,       kI,     kD,   Dir, chan
{},
{1, A9, 5, 0, ((float)0.1), 0, ((float)0.1), 1, 0},
{2, A7, 4, 1, ((float)0.1), 0, ((float)0.1), 0, 0},
{3, A8, 3, 2, ((float)0.1), 0, ((float)0.1), 1, 0},
{4, A9, 5, 0, ((float)0.1), 0, ((float)0.1), 1, 0},
{5, A7, 4, 1, ((float)0.1), 0, ((float)0.1), 0, 0},
{6, A8, 3, 2, ((float)0.1), 0, ((float)0.1), 1, 0},
{7, A9, 5, 0, ((float)0.1), 0, ((float)0.1), 1, 0},
{8, A7, 4, 1, ((float)0.1), 0, ((float)0.1), 0, 0},
{9, A8, 3, 2, ((float)0.1), 0, ((float)0.1), 1, 0},
{10, A9, 5, 0, ((float)0.1), 0, ((float)0.1), 1, 0},
{11, A7, 4, 1, ((float)0.1), 0, ((float)0.1), 0, 0},
{12, A8, 3, 2, ((float)0.1), 0, ((float)0.1), 1, 0}
};

#endif
