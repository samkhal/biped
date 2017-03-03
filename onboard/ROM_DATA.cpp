#include "ROM_DATA.h"

//ROM Memory Addresses
const int MEM_BYTES = 2;//sizeof(uint16_t)
int jointAddrOffset = 0;
int minPotAddrOffset = 0;
int maxPotAddrOffset = 0;
int zeroPotAddrOffset = 0;
int orientationAddrOffset = 0;

void calculateOffsets(int numOfJoints){
  jointAddrOffset = 0;
  minPotAddrOffset = numOfJoints*MEM_BYTES;
  maxPotAddrOffset = numOfJoints*MEM_BYTES + minPotAddrOffset;
  zeroPotAddrOffset = numOfJoints*MEM_BYTES + maxPotAddrOffset;
  orientationAddrOffset = numOfJoints*MEM_BYTES + zeroPotAddrOffset;
}

void ROM_allocate(int numOfJoints,std::vector<JointROM>& jointMem){
  calculateOffsets(numOfJoints);
  for (int i = 0; i<numOfJoints; i++){
    JointROM jROM;
    jROM.jointAddr = i*MEM_BYTES + jointAddrOffset;
    jROM.minPotAddr = i*MEM_BYTES + minPotAddrOffset;
    jROM.maxPotAddr = i*MEM_BYTES + maxPotAddrOffset;
    jROM.zeroPotAddr = i*MEM_BYTES + zeroPotAddrOffset;
    jROM.orientationAddr = i*MEM_BYTES + orientationAddrOffset;
    jointMem.push_back(jROM);
  }
}
