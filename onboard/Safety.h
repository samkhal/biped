#ifndef SAFETY_H_
#define SAFETY_H_
#include "biped_lcm/log_msg.hpp"
#include "log_util.hpp"

extern const int zeroTorque;
extern Logger logwarn;
class Joint;

class Safety{
  public:
    Safety(Joint joint_);
    int check(int drive);

  private:
    Joint* joint;
    int maxPot;
    int minPot;
};

#endif
