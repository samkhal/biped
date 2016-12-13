#ifndef ROM_DATA_H_
#define ROM_DATA_H_

//ROM Memory Addresses
const int MEM_BYTES = sizeof(uint16_t);
int jointAddrOffset = 0;
int minPotAddrOffset = 0;
int maxPotAddrOffset = 0;
int zeroPotAddrOffset = 0;
int orientationAddrOffset = 0;
const int numOfJoints = 3;

struct JointROM {
  int jointAddr;
  int minPotAddr;
  int maxPotAddr;
  int zeroPotAddr;
  int orientationAddr;
};

JointROM jointMem [numOfJoints];

void calculateOffsets(int numOfJoints){
  jointAddrOffset = 0;
  minPotAddrOffset = numOfJoints*MEM_BYTES;
  maxPotAddrOffset = numOfJoints*MEM_BYTES + minPotAddrOffset;
  zeroPotAddrOffset = numOfJoints*MEM_BYTES + maxPotAddrOffset;
  orientationAddrOffset = numOfJoints*MEM_BYTES + zeroPotAddrOffset;
}

void ROM_allocate(int numOfJoints){
  calculateOffsets(numOfJoints);
  for (int i = 0; i<numOfJoints; i++){
    JointROM jROM;
    jROM.jointAddr = i + jointAddrOffset;
    jROM.minPotAddr = i + minPotAddrOffset;
    jROM.maxPotAddr = i + maxPotAddrOffset;
    jROM.zeroPotAddr = i + zeroPotAddrOffset;
    jROM.orientationAddr = i + orientationAddrOffset;
    jointMem[i] = jROM;
  }
}

#endif
