//ROM Memory Addresses
const int MEM_BYTES = 2;
const uint8_t ROM_JOINTS[3] = {
  0, //link1Addr
  1, //link2Addr
  2 //link3Addr
};

const uint8_t ROM_ZERO_THETA[6] = {
  3, //zeroTheta1A
  4, //zeroTheta1B
  5, //zeroTheta2A
  6, //zeroTheta2B
  7, //zeroTheta3A
  8 //zeroTheta3B
};

const uint8_t ROM_MIN_POT[6] = {
  9, //pot1addrA
  10,//pot1addrB
  11,//pot2addrA
  12,//pot2addrB
  13,//pot3addrA
  14//pot3addrB
};

const uint8_t ROM_MAX_POT[6] = {
  15,//pot1addrC
  16,//pot1addrD
  17,//pot2addrC
  18,//pot2addrD
  19,//pot3addrC
  20//pot3addrD
};
const uint8_t ROM_ORIENTATION[6] = {
  21,//orient1A
  23,//orient2A
  25,//orient3A
};

const uint8_t ROM [27] = {
  0, //link1Addr
  1, //link2Addr
  2, //link3Addr
  3, //zeroTheta1A
  4, //zeroTheta1B
  5, //zeroTheta2A
  6, //zeroTheta2B
  7, //zeroTheta3A
  8, //zeroTheta3B
  9, //pot1addrA
  10,//pot1addrB
  11,//pot2addrA
  12,//pot2addrB
  13,//pot3addrA
  14,//pot3addrB
  15,//pot1addrC
  16,//pot1addrD
  17,//pot2addrC
  18,//pot2addrD
  19,//pot3addrC
  20,//pot3addrD
  21,//orient1A
  22,//orient1B
  23,//orient2A
  24,//orient2B
  25,//orient3A
  26//orient3B
};

enum ROM_ENUM {
  link1Addr = 0, //link1Addr
  link2Addr = 1,
  link3Addr = 2,
  zeroTheta1A = 3,
  zeroTheta1B = 4,
  zeroTheta2A = 5,
  zeroTheta2B = 6,
  zeroTheta3A = 7,
  zeroTheta3B = 8,
  pot1addrA = 9,
  pot1addrB = 10,
  pot2addrA = 11,
  pot2addrB = 12,
  pot3addrA = 13,
  pot3addrB = 14,
  pot1addrC = 15,
  pot1addrD = 16,
  pot2addrC = 17,
  pot2addrD = 18,
  pot3addrC = 19,
  pot3addrD = 20,
  orient1A = 21,
  orient1B = 22,
  orient2A = 23,
  orient2B = 24,
  orient3A = 25,
  orient3B = 26
};

enum ROM_ATTR{
  ZEROTHETA = 0,
  MIN_POT = 1,
  MAX_POT = 2,
  ORIENT = 3
};
