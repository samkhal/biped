#include "Safety.h"

Safety::Safety(Joint joint_) {
  //set min and max angle joints
  joint = &joint_;
  maxPot = joint->getMaxPot();
  minPot = joint->getMinPot();
  //compare to hard coded table?
}

int Safety::check(int drive){
  int current = joint->readPotentiometer();
  if(current < maxPot && current > minPot){
    return drive;
  }
  else{
    //logwarn << "Joint " << joint->getJointNumber() << " reached limit.\n";
    return zeroTorque;
  }
}
