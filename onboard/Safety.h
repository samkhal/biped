#ifndef SAFETY_H_
#define SAFETY_H_
#include "Joint.h"

extern int zeroTorque;
extern Logger logwarn;

class Safety{
  public:
    Safety(Joint joint_);
    int check(int drive);

  private:
    Joint joint;
    int maxPot;
    int minPot;
};

#endif
