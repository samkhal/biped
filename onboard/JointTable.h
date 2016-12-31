#include "Joint.h"

//Table of all robot's joints
const Joint JointTable[12] = {
//  Joint#, PotPin, MotorPin, EnablePin, kP,  kI,  kD,   Direction
Joint(1,   A9,       5,        0,      0.1f,   0,  0.1f,   1),
Joint(2,   A7,       4,        1,      0.1f,   0,  0.1f,   0),
Joint(3,   A8,       3,        2,      0.1f,   0,  0.1f,   1),
Joint(4,   A9,       5,        0,      0.1f,   0,  0.1f,   1),
Joint(5,   A7,       4,        1,      0.1f,   0,  0.1f,   0),
Joint(6,   A8,       3,        2,      0.1f,   0,  0.1f,   1),
Joint(7,   A9,       5,        0,      0.1f,   0,  0.1f,   1),
Joint(8,   A7,       4,        1,      0.1f,   0,  0.1f,   0),
Joint(9,   A8,       3,        2,      0.1f,   0,  0.1f,   1),
Joint(10,  A9,       5,        0,      0.1f,   0,  0.1f,   1),
Joint(11,  A7,       4,        1,      0.1f,   0,  0.1f,   0),
Joint(12,  A8,       3,        2,      0.1f,   0,  0.1f,   1)
};
