#ifndef ROM_DATA_H_
#define ROM_DATA_H_

#include <vector>

struct JointROM {
  int jointAddr;
  int minPotAddr;
  int maxPotAddr;
  int zeroPotAddr;
  int orientationAddr;
};

void calculateOffsets(int numOfJoints);

void ROM_allocate(int numOfJoints,std::vector<JointROM>& jointMem);

#endif
