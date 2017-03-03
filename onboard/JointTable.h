#include "Joint.h"

//Table of all robot's joints
const Joint JointTable[12] = {
//  Joint#, PotPin, MotorPin, EnablePin, kP,  kI,  kD,   motorOrient  potOriert   potCabling  OutOfRangeCheck
Joint(1,   A7,       4,        1,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Upper Left 0 - Hip X - Left Leg
Joint(2,   A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Upper Left 1 - Hip X - Right Leg
Joint(3,   A8,       3,        2,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Upper Left 2 - Hip Y - Left Leg
Joint(4,   A7,       3,        2,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Upper Right 0 - Hip Y - Right Leg
Joint(5,   A9,       4,        1,      0.1f,   0,  0.1f,      1,         1,           1,          false), //Upper Right 1 - Knee Y - Left Leg
Joint(6,   A8,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          false), //Upper Right 2 - Knee Y - Right Leg
Joint(7,   A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Right 0 - Ankle Y - Right Leg
Joint(8,   A7,       3,        2,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Right 1 - Ankle X - Right Leg
Joint(9,   A7,       3,        2,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Right 2 - Toe - Right Leg
Joint(10,  A9,       4,        1,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Left 0 - Ankle Y - Left Leg
Joint(11,  A7,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Left 1 - Ankle X - Left Leg
Joint(12,  A7,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true)  //Lower Left 2 - Toe - Left Leg
};
