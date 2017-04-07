#include "Joint.h"

//Table of all robot's joints
const Joint JointTable[12] = {
//  Joint#, PotPin, MotorPin, EnablePin, kP,  kI,  kD,   motorOrient  potOriert   potCabling  OutOfRangeCheck
Joint(1,   A9,       5,        0,      0.6f,   0,  0.1f,      1,         1,           1,          true), //Upper Left 0 - Hip X - Left Leg
Joint(2,   A7,       4,        1,      0.6f,   0,  0.1f,      1,         -1,           1,          true), //Upper Left 1 - Hip Y - Left Leg
Joint(3,   A8,       3,        2,      0.6f,   0,  0.1f,      1,         -1,           1,          false), //Upper Left 2 - Knee Y - Left Leg

Joint(4,   A9,       5,        0,      0.6f,   0,  0.1f,      1,         1,           1,          true), //Upper Right 0 - Hip X - Right Leg
Joint(5,   A7,       4,        1,      0.6f,   0,  0.1f,      -1,         1,           1,          true), //Upper Right 1 - Hip Y - Right Leg
Joint(6,   A8,       3,        2,      0.6f,   0,  0.1f,      -1,         -1,           1,          false), //Upper Right 2 - Knee Y - Right Leg

Joint(7,   A7,       4,        1,      0.1f,   0,  0.1f,      -1,         1,           1,          true), //Lower Left 0 - Ankle Y - Left Leg
Joint(8,   A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Left 1 - Ankle X - Left Leg
Joint(9,   A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          false), //Lower Left 2 - Toe - Left Leg

Joint(10,  A7,       4,        1,      0.1f,   0,  0.1f,      -1,         1,           1,          true), //Lower Right 0 - Ankle Y - Right Leg
Joint(11,  A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          true), //Lower Right 1 - Ankle X - Right Leg
Joint(12,  A9,       5,        0,      0.1f,   0,  0.1f,      1,         1,           1,          false)  //Lower Right 2 - Toe - Right Leg
};
