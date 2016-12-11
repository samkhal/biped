#ifndef JOINT_H_
#define JOINT_H_

struct Joint {
  int lastPID;
  int setPoint;
  int16_t *data;
};

#endif
