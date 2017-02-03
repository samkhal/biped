#include "Safety.h"

Safety::Safety(Joint joint_) : joint(joint_) {
  //set min and max angle joints
  maxPot = joint.getMaxPot();
  minPot = joint.getMinPot();
  //compare to hard coded table?
}

int Safety::check(int drive){
  int current = joint.readPotentiometer();
  if(current < maxPot && current > minPot){
    return drive;
  }
  else{
    logwarn << "Joint " << joint.getJointNumber() << " reached limit.\n";
    return zeroTorque;
  }
}
