#include <EEPROM.h>
#include <TimerThree.h>

//ROM Memory addresses
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
uint16_t a = 0;
volatile unsigned long int timer = 0;
volatile bool allowPD = true;
uint32_t dataLength = 0;
uint32_t cSum = 0;
int flag = 0;
uint32_t sumCounter = 0;

// Link 1 Variables
uint32_t *data1;

// Link 2 Variables
uint32_t *data2;

// Link 3 Variables
uint32_t *data3;

// PID Variables
int PIDPeriod = 10; // milliseconds
int ScaleFactor = 1;
int Motor;
int Motor1 = 5;
int Motor2 = 4;
int Motor3 = 3;
int Enable1 = 0;
int Enable2 = 1;
int Enable3 = 2;
int Position;
int Position1 = A9;
int Position2 = A7;
int Position3 = A8;
int Integral = 0;
float kP;
float kD;
float kP1 = 0.04;
float kP2 = 0.1;
float kP3 = 0.1;
float kD1 = 0.1;
float kD2 = 0.1;
float kD3 = 0.1;
int Last;
int Last1 = 0;
int Last2 = 0;
int Last3 = 0;
int IntThresh = 1;
int minPWM = 26;
int maxPWM = 229;
uint16_t minPot;
uint16_t maxPot;
int setPoint1;
int setPoint2;
int setPoint3;

uint32_t read_uint32() {
  while (Serial.available() < 4);
  uint32_t out = Serial.read();
  out += (Serial.read() << 8);
  out += (Serial.read() << 16);
  out += (Serial.read() << 24);
  return out;
}
uint16_t read_uint16() {
  while (Serial.available() < 2);
  uint16_t out = Serial.read();
  out += (Serial.read() << 8);
  return out;
}
void write_uint16(uint16_t val) {
  Serial.write((byte)(val));
  Serial.write((byte)(val >> 8));
}

void PIDcontrol(int SetPt, int link) {
  if (link == 1) {
    kP = kP1; kD = kD1; Position = Position1; Last = Last1; Motor = Motor1;
  } else if (link == 2) {
    kP = kP2; kD = kD2; Position = Position2; Last = Last2; Motor = Motor2;
  } else if (link == 3) {
    kP = kP3; kD = kD3; Position = Position3; Last = Last3; Motor = Motor3;
  }
  int Actual = analogRead(Position);
  int Error = SetPt - Actual;
  //  if (abs(Error) < IntThresh){ // prevent integral 'windup'
  //    Integral = Integral + Error; // accumulate the error integral
  //  }
  //  else {
  //  Integral=0; // zero it if out of bounds
  //  }
  float P = Error * kP; // calc proportional term
  //  float I = Integral*kI; // integral term
  float D = ((Last - Actual) * kD) / PIDPeriod; // derivative term
  int Drive = P ;//+ D; // Total drive = P+I+D
  if (link == 3){
    Drive = -Drive * ScaleFactor + (minPWM + maxPWM) / 2;
  }
  else{
    Drive = Drive * ScaleFactor + (minPWM + maxPWM) / 2; // scale Drive to be in the range 0-255
  }
  if (Drive < minPWM) { // Check which direction to go.
    Drive = minPWM;
  }
  if (Drive > maxPWM) {
    Drive = maxPWM;
  }
  analogWrite (Motor, Drive); // send PWM command to motor board
//  Serial.print(readROM(3,true));
//  Serial.print(" - ");
//  Serial.println(Drive);
//  write_uint16((uint16_t)Drive);
  if (link == 1) {
    Last1 = Actual;
  } else if (link == 2) {
    Last2 = Actual;
  } else if (link == 3) {
    Last3 = Actual;
  }
}

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

//Read ROM values
uint16_t minPot1 = readROM(1,true);
uint16_t maxPot1 = readROM(1,false);
uint16_t minPot2 = readROM(2,true);
uint16_t maxPot2 = readROM(2,false);
uint16_t minPot3 = readROM(3,true);
uint16_t maxPot3 = readROM(3,false);

bool Calibration(int link) {
  if (link == 1) {
    Position = Position1;
  } else if (link == 2) {
    Position = Position2;
  } else if (link == 3) {
    Position = Position3;
  }
  int potVal = analogRead(Position);
  if (minPot > (uint16_t)potVal) {
    minPot = (uint16_t)potVal;
  }
  else if (maxPot < (uint16_t)potVal) {
    maxPot = (uint16_t)potVal;
  }
  if (link == 1) {
    maxPot1 = maxPot; minPot1 = minPot;
    EEPROM.write(pot1addrA, (byte)(minPot1));
    EEPROM.write(pot1addrB, (byte)(minPot1 >> 8));
    EEPROM.write(pot1addrC, (byte)(maxPot1));
    EEPROM.write(pot1addrD, (byte)(maxPot1 >> 8));
  } else if (link == 2) {
    maxPot2 = maxPot; minPot2 = minPot;
    EEPROM.write(pot2addrA, (byte)(minPot2));
    EEPROM.write(pot2addrB, (byte)(minPot2 >> 8));
    EEPROM.write(pot2addrC, (byte)(maxPot2));
    EEPROM.write(pot2addrD, (byte)(maxPot2 >> 8));
  } else if (link == 3) {
    maxPot3 = maxPot; minPot3 = minPot;
    EEPROM.write(pot3addrA, (byte)(minPot3));
    EEPROM.write(pot3addrB, (byte)(minPot3 >> 8));
    EEPROM.write(pot3addrC, (byte)(maxPot3));
    EEPROM.write(pot3addrD, (byte)(maxPot3 >> 8));
  }
  if (potVal < 10 || potVal > 1010) {
    return false;
  }
  return true;
}

void runTrajectory(int link) {
  uint32_t i = 0;
  digitalWrite(Enable1, HIGH);
  digitalWrite(Enable2, HIGH);
  digitalWrite(Enable3, HIGH);
  while(i<dataLength){
    if (!allowPD) {
      i = i - 1;
    }
    else if(allowPD) {
      allowPD = false;
      if (link == 4) {
        PIDcontrol((int)data1[i], 1);
        PIDcontrol((int)data2[i], 2);
        PIDcontrol((int)data3[i], 3);
      }
      else if (link == 3) {
        PIDcontrol((int)data3[i], link);
      }
      else if (link == 2) {
        PIDcontrol((int)data2[i], link);
      }
      else if (link == 1) {
        PIDcontrol((int)data1[i], link);
      }
    }
    i = i+1;
  }
  digitalWrite(Enable1, LOW);
  digitalWrite(Enable2, LOW);
  digitalWrite(Enable3, LOW);
}

bool checkOOR(int Position, bool OutOfRange) {
  int pose = analogRead(Position);
  int i = 0;
  if (Position == Position1){
    i = 1;
  }
  else if (Position == Position2){
    i = 2;
  }
  else if (Position == Position3){
    i = 3;
  }
  if (pose < ((int)readROM(i, true)+3) || pose > ((int)readROM(i, false)-3)) {
    return true;
  }
  return false;
}

void timerCallback() {
  timer++;
  if (timer >= 10) { //every 10 ms
    allowPD = true;
    timer = 0;
  }
}

void setup() {
  Serial.begin(115200);
  setPoint1 = analogRead(Position1);
  setPoint2 = analogRead(Position2);
  setPoint3 = analogRead(Position3);
  pinMode(Motor1, OUTPUT);
  pinMode(Motor2, OUTPUT);
  pinMode(Motor3, OUTPUT);
  pinMode(Enable1, OUTPUT);
  pinMode(Enable2, OUTPUT);
  pinMode(Enable3, OUTPUT);
  analogWrite(Motor1, 127);
  analogWrite(Motor2, 127);
  analogWrite(Motor3, 127);
  digitalWrite(Enable1, LOW);
  digitalWrite(Enable2, LOW);
  digitalWrite(Enable3, LOW);
  Timer3.initialize(1000); //1 ms
  Timer3.attachInterrupt(timerCallback);
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
      action = Serial.read();
      //        write_uint16((uint16_t)action);
      link = Serial.read();
      if (action == (uint8_t)1){
        dataLength = read_uint32();
        cSum = read_uint32();
      }
      else{
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
      runTrajectory(link);
      write_uint16(3);
      break;

    case (uint8_t)10  : // startCalibration
      if (link == 1) {
        minPot = (uint16_t)setPoint1;
        maxPot = (uint16_t)setPoint1;
      } else if (link == 2) {
        minPot = (uint16_t)setPoint2;
        maxPot = (uint16_t)setPoint2;
      } else if (link == 3) {
        minPot = (uint16_t)setPoint3;
        maxPot = (uint16_t)setPoint3;
      }
      bool stopCalFlag;
      bool calibrationOutOfRange;
      stopCalFlag = true;
      calibrationOutOfRange = false;
      write_uint16((uint16_t)0);
      while (stopCalFlag) {
        if (Calibration(link) == false) {
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
      setPoint1 = analogRead(Position1);
      setPoint2 = analogRead(Position2);
      setPoint3 = analogRead(Position3);
      // setPoint = 250;/////////////////////////////////////////////////
      digitalWrite(Enable1, HIGH);
      digitalWrite(Enable2, HIGH);
      digitalWrite(Enable3, HIGH);
      while (!stopStaticControl && !OutOfRange) {
        PIDcontrol(setPoint1, 1);
        PIDcontrol(setPoint2, 2);
        PIDcontrol(setPoint3, 3);
        if (Serial.available() >= 1) {
          if ((int)Serial.read() == (int)13) { // stopStaticControl
            stopStaticControl = true;
          }
        }
        OutOfRange = checkOOR(Position1, OutOfRange)||checkOOR(Position2, OutOfRange)||checkOOR(Position3, OutOfRange);
      }
      digitalWrite(Enable1, LOW);
      digitalWrite(Enable2, LOW);
      digitalWrite(Enable3, LOW);
      write_uint16((uint16_t)0);
      break;
    case (uint8_t)1   : // sendTrajectory
      write_uint16(dataLength);
      if (link == 1) {
        data1 = (uint32_t *)malloc(dataLength * sizeof(uint32_t));
      }
      else if (link == 2) {
        data2 = (uint32_t *)malloc(dataLength * sizeof(uint32_t));
      }
      else if (link == 3) {
        data3 = (uint32_t *)malloc(dataLength * sizeof(uint32_t));
      }

      for (uint32_t i = 0; i < dataLength; i++) {
        if (Serial.available() >= 2) {
          a = read_uint16();
          if (link == 1) {
            data1[i] = a;
          }
          else if (link == 2) {
            data2[i] = a;
          }
          else if (link == 3) {
            data3[i] = a;
          }
        }
        else {
          i = i - 1;
        }
      }
      for (uint32_t i = 0; i < dataLength; i++) {
        if (link == 1) {
          sumCounter = sumCounter + data1[i];
        }
        else if (link == 2) {
          sumCounter = sumCounter + data2[i];
        }
        else if (link == 3) {
          sumCounter = sumCounter + data3[i];
        }
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
      minData = readROM(link,true);
      maxData = readROM(link,false);
      write_uint16(minData);
      write_uint16(maxData);
      write_uint16((uint16_t)0);
    default :
      delay(10);
  }
}


