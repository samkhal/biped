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

extern void calculateOffsets(int numOfJoints);

extern std::vector<JointROM> ROM_allocate(int numOfJoints,std::vector<JointROM> jointMem);

#endif
