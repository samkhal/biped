#include <EEPROM.h>
#include <TimerThree.h>
#include <stdio.h>
#include <string.h>

//ROM Memory addresses
unsigned int link1Addr = 0;
unsigned int link2Addr = 1;
unsigned int link3Addr = 2;
unsigned int zeroTheta1A = 3;
unsigned int zeroTheta1B = 4;
unsigned int zeroTheta2A = 5;
unsigned int zeroTheta2B = 6;
unsigned int zeroTheta3A = 7;
unsigned int zeroTheta3B = 8;
unsigned int pot1addrA = 9;
unsigned int pot1addrB = 10;
unsigned int pot1addrC = 11;
unsigned int pot1addrD = 12;
unsigned int pot2addrA = 13;
unsigned int pot2addrB = 14;
unsigned int pot2addrC = 15;
unsigned int pot2addrD = 16;
unsigned int pot3addrA = 17;
unsigned int pot3addrB = 18;
unsigned int pot3addrC = 19;
unsigned int pot3addrD = 20;
unsigned int orient1 = 21;
unsigned int orient2 = 22;
unsigned int orient3 = 23;
unsigned int zeroAngleAddr = 24;


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
  int16_t *data;
};

typedef struct ConstJoint {
  int link;
  int position;
  int motor;
  int enable;
  float kP;
  float kI;
  float kD;
  int direction;
  char name[10];
  int lastPID;
  int minPot;
  int maxPot;
  int minTheta; // in rads values
  int maxTheta; // in rads values
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
int readROM(int link, int attribute) {
  int address1;
  int address2;
  if (link == 1) {
    if (attribute == 0) {
      address1 = pot1addrA;
      address2 = pot1addrB;
    }
    else if (attribute == 1) {
      address1 = pot1addrC;
      address2 = pot1addrD;
    }
    else if (attribute == 3) {
      address1 = zeroTheta1A;
      address2 = zeroTheta1B;
    }
    else if (attribute == 4) {
      address1 = orient1;
      address2 = zeroAngleAddr;
    }
  }
  if (link == 2) {
    if (attribute == 0) {
      address1 = pot2addrA;
      address2 = pot2addrB;
    }
    else if (attribute == 1) {
      address1 = pot2addrC;
      address2 = pot2addrD;
    }
    else if (attribute == 3) {
      address1 = zeroTheta2A;
      address2 = zeroTheta2B;
    }
    else if (attribute == 4) {
      address1 = orient2;
      address2 = zeroAngleAddr;
    }
  }
  if (link == 3) {
    if (attribute == 0) {
      address1 = pot3addrA;
      address2 = pot3addrB;
    }
    else if (attribute == 1) {
      address1 = pot3addrC;
      address2 = pot3addrD;
    }
    else if (attribute == 3) {
      address1 = zeroTheta3A;
      address2 = zeroTheta3B;
    }
    else if (attribute == 4) {
      address1 = orient3;
      address2 = zeroAngleAddr;
    }
  }
  int val = EEPROM.read(address1);
  val = val + (EEPROM.read(address2) << 8);
  return val;
}

//Declaring the joint structs, needs to be changed to link1-3
//                    L, P,  M, E, kP,  kI, kD
ConstJoint link1_c = {1, A9, 5, 0, ((float)0.1), 0, ((float)0.1), 1};
ConstJoint link2_c = {2, A7, 4, 1, ((float)0.1), 0, ((float)0.1), 0};
ConstJoint link3_c = {3, A8, 3, 2, ((float)0.1), 0, ((float)0.1), 0};
ConstJoint* link1_cp;
ConstJoint* link2_cp;
ConstJoint* link3_cp;

Joint link1;
Joint link2;
Joint link3;
Joint* link1_p;
Joint* link2_p;
Joint* link3_p;

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
  int sign = 1;
  if(cjoint.direction == 0){
    sign = -1;
  }
  Drive = (sign * Drive * ScaleFactor + (minPWM + maxPWM) / 2); // scale Drive to be in the range 0-255
  if (Drive < minPWM) { // Check which direction to go.
    Drive = minPWM;
  }
  if (Drive > maxPWM) {
    Drive = maxPWM;
  }
//  write_uint16((uint16_t) Drive);
  analogWrite (cjoint.motor, Drive); // send PWM command to motor board
  
  joint->lastPID = Actual;
}

void setOrientROM(ConstJoint* cjoint){
  if (cjoint->link == 1){
    EEPROM.write(orient1, (byte)(cjoint->direction));
  }
  else if (cjoint->link == 2){
    EEPROM.write(orient2, (byte)(cjoint->direction));
  }
  else if (cjoint->link == 3){
    EEPROM.write(orient3, (byte)(cjoint->direction));
  }
  EEPROM.write(zeroAngleAddr, (byte)0);
}

void setZeroThetaROM(ConstJoint* cjoint){
  int val = 0;
  val = analogRead(cjoint->position);
  if (cjoint->link == 1){
    EEPROM.write(zeroTheta1A, (byte)(val));
    EEPROM.write(zeroTheta1B, (byte)(val >> 8));
  }
  else if (cjoint->link == 2){
    EEPROM.write(zeroTheta2A, (byte)(val));
    EEPROM.write(zeroTheta2B, (byte)(val >> 8));
  }
  else if (cjoint->link == 3){
    EEPROM.write(zeroTheta3A, (byte)(val));
    EEPROM.write(zeroTheta3B, (byte)(val >> 8));
  }
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
  digitalWrite(link1_c.enable, HIGH);
  digitalWrite(link2_c.enable, HIGH);
  digitalWrite(link3_c.enable, HIGH);
  while (i < dataLength) {
    if (!allowPD) {
      i = i - 1;
    }
    else if (allowPD) {
      allowPD = false;
      if (link == 4) {
        PIDcontrol((int)radsToPot(joint_p->data[i],cjoint), link1_p, link1_c);
        PIDcontrol((int)radsToPot(joint_p->data[i],cjoint), link2_p, link2_c);
        PIDcontrol((int)radsToPot(joint_p->data[i],cjoint), link3_p, link3_c);
      }
      else {
        int16_t val = radsToPot(joint_p->data[i],cjoint);
        PIDcontrol(val, joint_p, cjoint);
      }
    }
    i = i + 1;
  }
  digitalWrite(link1_c.enable, LOW);
  digitalWrite(link2_c.enable, LOW);
  digitalWrite(link3_c.enable, LOW);
}

//Checks if a joint goes very close to the min/max values and it stops it
bool checkOOR(ConstJoint cjoint) {
  int pose = analogRead(cjoint.position);
  int i = cjoint.link;
  if (pose < ((int)readROM(i, 0) + 3) || pose > ((int)readROM(i, 1) - 3)) {
    return true;
  }
  return false;
}

//Takes a pointer of a Joint struct and it changes its data from rads to pot values
int radsToPot(int16_t data, ConstJoint cjoint){
  int ticksPerRad = 172;// 3 ticks per degree
  int radsMultiplier = 1000;
  return ((data*ticksPerRad)/radsMultiplier + cjoint.zeroTheta);
}

float potToRads(int val, ConstJoint cjoint){
  float out;
  int ticksPerRad = 172;// 3 ticks per degree
  int radsMultiplier = 1000;
  val = val-cjoint.zeroTheta;
  out = (val/ticksPerRad)*radsMultiplier;
  return out;
}

void setup() {
  Serial.begin(115200);
  link1.setPoint = analogRead(link1_c.position);
  link2.setPoint = analogRead(link2_c.position);
  link3.setPoint = analogRead(link3_c.position);
  pinMode(link1_c.motor, OUTPUT);
  pinMode(link2_c.motor, OUTPUT);
  pinMode(link3_c.motor, OUTPUT);
  pinMode(link1_c.enable, OUTPUT);
  pinMode(link2_c.enable, OUTPUT);
  pinMode(link3_c.enable, OUTPUT);
  analogWrite(link1_c.motor, 127);
  analogWrite(link2_c.motor, 127);
  analogWrite(link3_c.motor, 127);
  digitalWrite(link1_c.enable, LOW);
  digitalWrite(link2_c.enable, LOW);
  digitalWrite(link3_c.enable, LOW);
  Timer3.initialize(1000); //1 ms
  Timer3.attachInterrupt(timerCallback);

  link1_cp = &link1_c;
  link2_cp = &link2_c;
  link3_cp = &link3_c;
  link1_p = &link1;
  link2_p = &link2;
  link3_p = &link3;
  strcpy(link1_c.name, "LINK1");
  strcpy(link2_c.name, "LINK2");
  strcpy(link3_c.name, "LINK3");
  EEPROM.write(link1Addr, (byte)(7));
  EEPROM.write(link2Addr, (byte)(8));
  EEPROM.write(link3Addr, (byte)(9));
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
        joint = link1; cjoint = link1_c; joint_p = link1_p; cjoint_p = link1_cp;
      } else if (link == 2) {
        joint = link2; cjoint = link2_c; joint_p = link2_p; cjoint_p = link2_cp;
      } else if (link == 3) {
        joint = link3; cjoint = link3_c; joint_p = link3_p; cjoint_p = link3_cp;
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
      minPot = (uint16_t)analogRead(cjoint_p->position);
      maxPot = minPot;
      cjoint_p->minPot = (uint16_t)joint_p->setPoint;
      cjoint_p->maxPot = (uint16_t)joint_p->setPoint;
      cjoint_p->zeroTheta = (uint16_t)analogRead(cjoint_p->position);
      setOrientROM(cjoint_p);
      setZeroThetaROM(cjoint_p);
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
        write_uint16(cjoint_p->direction);
        write_uint16(cjoint_p->zeroTheta);
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
      link1_p->setPoint = analogRead(link1_c.position);
      link2_p->setPoint = analogRead(link2_c.position);
      link3_p->setPoint = analogRead(link3_c.position);
      // setPoint = 250;/////////////////////////////////////////////////
      digitalWrite(link1_c.enable, HIGH);
      digitalWrite(link2_c.enable, HIGH);
      digitalWrite(link3_c.enable, HIGH);
      while (!stopStaticControl && !OutOfRange) {
        PIDcontrol(link1_p->setPoint, link1_p, link1_c);
        PIDcontrol(link2_p->setPoint, link2_p, link2_c);
        PIDcontrol(link3_p->setPoint, link3_p, link3_c);
        if (Serial.available() >= 1) {
          if ((int)Serial.read() == (int)13) { // stopStaticControl
            stopStaticControl = true;
          }
        }
//        OutOfRange = checkOOR(link1_c)||checkOOR(link2_c);
      }
      digitalWrite(link1_c.enable, LOW);
      digitalWrite(link2_c.enable, LOW);
      digitalWrite(link3_c.enable, LOW);
      write_uint16((uint16_t)0);
      break;
    case (uint8_t)1   : // sendTrajectory
      write_uint16(dataLength);
      joint_p->data = (int16_t *)malloc(dataLength * sizeof(int16_t));

      for (uint32_t i = 0; i < dataLength; i++) {
        if (Serial.available() >= 2) {
          a = read_int16();
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

    case (uint8_t)11   : // get orientation,zero angle, min & max pot values
      uint16_t minData;
      uint16_t maxData;
      uint16_t orientation;
      uint16_t zeroAngle;
      minData = readROM(link, 0);
      maxData = readROM(link, 1);
      zeroAngle = readROM(link, 3);
      orientation = readROM(link, 4);
      cjoint_p->minPot = (int)minData;
      cjoint_p->maxPot = (int)maxData;
      cjoint_p->zeroTheta = (int)zeroAngle;
      write_uint16(orientation);
      write_uint16(zeroAngle);
      write_uint16(minData);
      write_uint16(maxData);
      write_uint16((uint16_t)0);
      break;
      
    case (uint8_t)15   : // get links associated with this Teensy
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


