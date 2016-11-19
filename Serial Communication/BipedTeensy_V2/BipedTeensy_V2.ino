#include <EEPROM.h>
#include <TimerThree.h>
#include <stdio.h>
#include <string.h>

//ROM Memory addresses
unsigned int link1Addr = 0;
unsigned int link2Addr = 1;
unsigned int link3Addr = 2;
unsigned int pot1addrA = 8;
unsigned int pot1addrB = 9;
unsigned int pot1addrC = 10;
unsigned int pot1addrD = 11;
unsigned int pot2addrA = 12;
unsigned int pot2addrB = 13;
unsigned int pot2addrC = 14;
unsigned int pot2addrD = 15;
unsigned int pot3addrA = 16;
unsigned int pot3addrB = 17;
unsigned int pot3addrC = 18;
unsigned int pot3addrD = 19;

// Serial Communication variables
uint8_t action = 0;
uint8_t link = 0;
uint16_t a = 0; //variable for storing random uint16 numbers
volatile unsigned long int timer = 0; 
volatile bool allowPD = true; //flag for allowing the control
uint32_t dataLength = 0; // amount of data to be received
uint32_t cSum = 0; // checksum of data to be received 
int flag = 0; //flag for the main loop 
uint32_t sumCounter = 0; //variabl that sums the bytes received

// PID Variables
int PIDPeriod = 10; // milliseconds
int ScaleFactor = 1;
int IntThresh = 1;
int minPWM = 26;
int maxPWM = 229;
uint16_t minPot;
uint16_t maxPot;

typedef struct Joint {
  int lastPID;
  int setPoint;
  uint16_t *data;
};

typedef struct ConstJoint {
  int teensy;
  int link;
  int position;
  int motor;
  int enable;
  int lastPID;
  int kP;
  int kI;
  int kD;
  int direction;
  char name[10];
  int minPot;
  int maxPot;
  int minTheta; // in pot values
  int maxTheta; // in pot values
  int zeroTheta; // in pot values
};

//Timer interrupt callback, increases the timer variable and sets the flag ready for PID control
void timerCallback() {
  timer++;
  if (timer >= 10) { //every 10 ms
    allowPD = true;
    timer = 0;
  }
}

//reads 4 unsigned bytes of serial data
uint32_t read_uint32() {
  while (Serial.available() < 4);
  uint32_t out = Serial.read();
  out += (Serial.read() << 8);
  out += (Serial.read() << 16);
  out += (Serial.read() << 24);
  return out;
}

//reads 2 unsigned bytes of serial data
uint16_t read_uint16() {
  while (Serial.available() < 2);
  uint16_t out = Serial.read();
  out += (Serial.read() << 8);
  return out;
}

//reads 2 signed bytes of serial data
uint16_t read_int16() {
  while (Serial.available() < 2);
  int16_t out = Serial.read();
  out += (Serial.read() << 8);
  return out;
}

//sends 2 unsigned bytes to serial
void write_uint16(uint16_t val) {
  Serial.write((byte)(val));
  Serial.write((byte)(val >> 8));
}

//takes number of potentiometer(1-3) and a boolean if it is the minimum value
//returns the ROM memory of min/max range for that potentiometer
int readROM(int Pot, bool Min) {
  int address1;
  int address2;
  if (Pot == 1) {
    if (Min == true) {
      address1 = pot1addrA;
      address2 = pot1addrB;
    }
    else if (Min == false) {
      address1 = pot1addrC;
      address2 = pot1addrD;
    }
  }
  if (Pot == 2) {
    if (Min == true) {
      address1 = pot2addrA;
      address2 = pot2addrB;
    }
    else if (Min == false) {
      address1 = pot2addrC;
      address2 = pot2addrD;
    }
  }
  if (Pot == 3) {
    if (Min == true) {
      address1 = pot3addrA;
      address2 = pot3addrB;
    }
    else if (Min == false) {
      address1 = pot3addrC;
      address2 = pot3addrD;
    }
  }
  int val = EEPROM.read(address1);
  val = val + (EEPROM.read(address2) << 8);
  return val;
}

//Declaring the joint structs, needs to be changed to link1-3
//                 T, L, P,  M, E, kP,  kI, kD, mPot, MPot, stPt
ConstJoint hip_c = {1, 1, A9, 5, 0, 0.04, 0, 0.1, 1};
ConstJoint upperLeg_c = {1, 2, A7, 4, 1, 0.1, 0, 0.1, 1};
ConstJoint knee_c = {1, 3, A8, 3, 2, 0.1, 0, 0.1, -1};
ConstJoint* hip_cp;
ConstJoint* upperLeg_cp;
ConstJoint* knee_cp;

Joint hip;
Joint upperLeg;
Joint knee;
Joint* hip_p;
Joint* upperLeg_p;
Joint* knee_p;

Joint joint;
Joint* joint_p;
ConstJoint cjoint;
ConstJoint* cjoint_p;

//PID control, takes a setpoint in pot values, a joint struct pointer and a constjoint struct (of the same joint)
void PIDcontrol(int setPoint, Joint* joint, ConstJoint cjoint) {
  int Actual = analogRead(cjoint.position);
  int Error = setPoint - Actual;
  //  if (abs(Error) < IntThresh){ // prevent integral 'windup'
  //    Integral = Integral + Error; // accumulate the error integral
  //  }
  //  else {
  //  Integral=0; // zero it if out of bounds
  //  }
  float P = Error * cjoint.kP; // calc proportional term
  //  float I = Integral*kI; // integral term
  float D = ((joint->lastPID - Actual) * cjoint.kD) / PIDPeriod; // derivative term
  int Drive = P ;//+ D; // Total drive = P+I+D
  Drive = cjoint.direction * Drive * ScaleFactor + (minPWM + maxPWM) / 2; // scale Drive to be in the range 0-255
  if (Drive < minPWM) { // Check which direction to go.
    Drive = minPWM;
  }
  if (Drive > maxPWM) {
    Drive = maxPWM;
  }
  analogWrite (cjoint.motor, Drive); // send PWM command to motor board
  joint->lastPID = Actual;
}

//Writes the the maximum and minimum pot values to ROM, takes a pointer of constJoint struct
bool Calibration(ConstJoint* cjoint) {
  int potVal = analogRead(cjoint->position);
  if (minPot > (uint16_t)potVal) {
    minPot = (uint16_t)potVal;
  }
  else if (maxPot < (uint16_t)potVal) {
    maxPot = (uint16_t)potVal;
  }
  cjoint->minPot = minPot;
  cjoint->maxPot = maxPot;
  if (cjoint->link == 1) {
    EEPROM.write(pot1addrA, (byte)(cjoint->minPot));
    EEPROM.write(pot1addrB, (byte)(cjoint->minPot >> 8));
    EEPROM.write(pot1addrC, (byte)(cjoint->maxPot));
    EEPROM.write(pot1addrD, (byte)(cjoint->maxPot >> 8));
  } else if (cjoint->link == 2) {
    EEPROM.write(pot2addrA, (byte)(cjoint->minPot));
    EEPROM.write(pot2addrB, (byte)(cjoint->minPot >> 8));
    EEPROM.write(pot2addrC, (byte)(cjoint->maxPot));
    EEPROM.write(pot2addrD, (byte)(cjoint->maxPot >> 8));
  } else if (cjoint->link == 3) {
    EEPROM.write(pot3addrA, (byte)(cjoint->minPot));
    EEPROM.write(pot3addrB, (byte)(cjoint->minPot >> 8));
    EEPROM.write(pot3addrC, (byte)(cjoint->maxPot));
    EEPROM.write(pot3addrD, (byte)(cjoint->maxPot >> 8));
  }
  if (potVal < 10 || potVal > 1010) {
    return false;
  }
  return true;
}

//Runs the trajectory received, takes a pointer of a joint struct and a constJoint struct
void runTrajectory(Joint* joint_p, ConstJoint cjoint) {
  uint32_t i = 0;
  digitalWrite(hip_c.enable, HIGH);
  digitalWrite(upperLeg_c.enable, HIGH);
  digitalWrite(knee_c.enable, HIGH);
  while (i < dataLength) {
    if (!allowPD) {
      i = i - 1;
    }
    else if (allowPD) {
      allowPD = false;
      if (link == 4) {
        PIDcontrol((int)hip_p->data[i], hip_p, hip_c);
        PIDcontrol((int)upperLeg_p->data[i], upperLeg_p, upperLeg_c);
        PIDcontrol((int)knee_p->data[i], knee_p, knee_c);
      }
      else {
        PIDcontrol((int)joint_p->data[i], joint_p, cjoint);
      }
    }
    i = i + 1;
  }
  digitalWrite(hip_c.enable, LOW);
  digitalWrite(upperLeg_c.enable, LOW);
  digitalWrite(knee_c.enable, LOW);
}

//Checks if a joint goes very close to the min/max values and it stops it
bool checkOOR(ConstJoint cjoint) {
  int pose = analogRead(cjoint.position);
  int i = cjoint.link;
  if (pose < ((int)readROM(i, true) + 3) || pose > ((int)readROM(i, false) - 3)) {
    return true;
  }
  return false;
}

//Takes a pointer of a Joint struct and it changes its data from rads to pot values
void radsToPot(Joint* joint){
  int ticksPerRad = 1;
  int radsMultiplier = 1000;
  for (int i = 0; i<sizeof(joint->data); i++){
      joint->data[i] = (joint->data[i]*ticksPerRad)/radsMultiplier;
  }
}

void setup() {
  Serial.begin(115200);
  hip.setPoint = analogRead(hip_c.position);
  upperLeg.setPoint = analogRead(upperLeg_c.position);
  knee.setPoint = analogRead(knee_c.position);
  pinMode(hip_c.motor, OUTPUT);
  pinMode(upperLeg_c.motor, OUTPUT);
  pinMode(knee_c.motor, OUTPUT);
  pinMode(hip_c.enable, OUTPUT);
  pinMode(upperLeg_c.enable, OUTPUT);
  pinMode(knee_c.enable, OUTPUT);
  analogWrite(hip_c.motor, 127);
  analogWrite(upperLeg_c.motor, 127);
  analogWrite(knee_c.motor, 127);
  digitalWrite(hip_c.enable, LOW);
  digitalWrite(upperLeg_c.enable, LOW);
  digitalWrite(knee_c.enable, LOW);
  Timer3.initialize(1000); //1 ms
  Timer3.attachInterrupt(timerCallback);

  hip_cp = &hip_c;
  upperLeg_cp = &upperLeg_c;
  knee_cp = &knee_c;
  hip_p = &hip;
  upperLeg_p = &upperLeg;
  knee_p = &knee;
  strcpy(hip_c.name, "HIP");
  strcpy(upperLeg_c.name, "UPPERLEG");
  strcpy(knee_c.name, "KNEE");
  EEPROM.write(link1Addr, (byte)(10));
  EEPROM.write(link2Addr, (byte)(11));
  EEPROM.write(link3Addr, (byte)(12));
}
/* Header:
  1 byte: message type
  0xFF: Start running
  0x01: Send trajectory
  4-byte data length
  4-byte checksum
*/

// Main loop
void loop() {
  flag = 0;
  action = 0;
  link = 0;
  cSum = 0;
  sumCounter = 0;

  // Get initilization bytes
  while (flag == 0) {
    if (Serial.available() >= 10) {
      action = Serial.read(); // variable used in the state machine
      link = Serial.read(); // what joint/link to control
      if (link == 1) {
        joint = hip; cjoint = hip_c; joint_p = hip_p; cjoint_p = hip_cp;
      } else if (link == 2) {
        joint = upperLeg; cjoint = upperLeg_c; joint_p = upperLeg_p; cjoint_p = upperLeg_cp;
      } else if (link == 3) {
        joint = knee; cjoint = knee_c; joint_p = knee_p; cjoint_p = knee_cp;
      }
      if (action == (uint8_t)1) {
        dataLength = read_uint32();
        cSum = read_uint32();
      }
      else {
        a = read_uint32();
        a = read_uint32();
      }
      flag = 1;
    }
  }

  // State Machine
  switch (action) {
    case (uint8_t)255  : // runTrajectory
      write_uint16(1);
      runTrajectory(joint_p, cjoint);
      write_uint16(3);
      break;

    case (uint8_t)10  : // startCalibration
      cjoint_p->minPot = (uint16_t)joint_p->setPoint;
      cjoint_p->maxPot = (uint16_t)joint_p->setPoint;
      bool stopCalFlag;
      bool calibrationOutOfRange;
      stopCalFlag = true;
      calibrationOutOfRange = false;
      write_uint16((uint16_t)0);
      while (stopCalFlag) {
        if (Calibration(cjoint_p) == false) {
          calibrationOutOfRange = true;
        }
        if (Serial.available() >= 10) {
          if ((int)Serial.read() == (int)11) { // stopCalibration
            stopCalFlag = false;
            a = read_uint32();
            a = read_uint32();
            a = Serial.read();
          }
        }
      }
      if (calibrationOutOfRange == true) {
        write_uint16((uint16_t)666);
        write_uint16((uint16_t)0);
      }
      else {
        write_uint16(minPot);
        write_uint16(maxPot);
        write_uint16((uint16_t)0);
      }
      break;

    case (uint8_t)12  : // runStaticControl
      write_uint16((uint16_t)0);
      delay(100);
      bool stopStaticControl;
      bool OutOfRange;
      stopStaticControl = false;
      OutOfRange = false;
      hip_p->setPoint = analogRead(hip_c.position);
      upperLeg_p->setPoint = analogRead(upperLeg_c.position);
      knee_p->setPoint = analogRead(knee_c.position);
      // setPoint = 250;/////////////////////////////////////////////////
      digitalWrite(hip_c.enable, HIGH);
      digitalWrite(upperLeg_c.enable, HIGH);
      digitalWrite(knee_c.enable, HIGH);
      while (!stopStaticControl && !OutOfRange) {
        PIDcontrol(hip_p->setPoint, hip_p, hip_c);
        PIDcontrol(upperLeg_p->setPoint, upperLeg_p, upperLeg_c);
        PIDcontrol(knee_p->setPoint, knee_p, knee_c);
        if (Serial.available() >= 1) {
          if ((int)Serial.read() == (int)13) { // stopStaticControl
            stopStaticControl = true;
          }
        }
        OutOfRange = checkOOR(hip_c) || checkOOR(upperLeg_c) || checkOOR(knee_c);
      }
      digitalWrite(hip_c.enable, LOW);
      digitalWrite(upperLeg_c.enable, LOW);
      digitalWrite(knee_c.enable, LOW);
      write_uint16((uint16_t)0);
      break;
    case (uint8_t)1   : // sendTrajectory
      write_uint16(dataLength);
      joint_p->data = (uint16_t *)malloc(dataLength * sizeof(uint16_t));

      for (uint32_t i = 0; i < dataLength; i++) {
        if (Serial.available() >= 2) {
          a = read_uint16();
          joint_p->data[i] = a;
        }
        else {
          i = i - 1;
        }
      }
      for (uint32_t i = 0; i < dataLength; i++) {
        sumCounter = sumCounter + joint_p->data[i];
      }
      if (sumCounter == cSum) {
        a = 0;
        write_uint16(a);
      }
      else {
        a = 2;
        write_uint16(a);
      }
      break;

    case (uint8_t)11   : // get min/max pot values
      uint16_t minData;
      uint16_t maxData;
      minData = readROM(link, true);
      maxData = readROM(link, false);
      write_uint16(minData);
      write_uint16(maxData);
      write_uint16((uint16_t)0);
      
    case (uint8_t)14   : // get links associated with this Teensy
      uint16_t link1;
      uint16_t link2;
      uint16_t link3;
      link1 = EEPROM.read(link1Addr);
      link2 = EEPROM.read(link2Addr);
      link3 = EEPROM.read(link3Addr);
      write_uint16(link1);
      write_uint16(link2);
      write_uint16(link3);
      write_uint16((uint16_t)0);
      break;
    default :
      delay(10);
  }
}


