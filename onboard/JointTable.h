#include "Joint.h"

//Table of all robot's joints
const Joint JointTable[12] = {
//  Joint#, PotPin, MotorPin, EnablePin, kP,  kI,  kD,   motorOrient  potOriert   potCabling  OutOfRangeCheck
Joint(1,   A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Upper Right 0 - Hip X
Joint(2,   A8,       4,        1,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Upper Right 1 - Hip Y
Joint(3,   A7,       3,        2,      0.1f,   0,  0.1f,      1,         1,           1,          false), //Upper Right 2 - Knee Y
Joint(4,   A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Upper Left 0 - Hip X
Joint(5,   A7,       4,        1,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Upper Left 1 - Hip Y
Joint(6,   A8,       3,        2,      0.1f,   0,  0.1f,      1,         1,           1,          false), //Upper Left 2 - Knee Y
Joint(7,   A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Right 0 - Ankle Y
Joint(8,   A7,       4,        1,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Right 1 - Ankle X
Joint(9,   A8,       3,        2,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Right 2 - Toe
Joint(10,  A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Left 0 - Ankle Y
Joint(11,  A7,       4,        1,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Left 1 - Ankle X
Joint(12,  A8,       3,        2,      0.1f,   0,  0.1f,      1,         1,           1,          true)  //Lower Left 2 - Toe
};
