#include "WProgram.h"

// Serial Communication variables
uint8_t action = 0;
uint32_t cSum = 0;
uint32_t dataLength = 0;
uint16_t a = 0;
uint32_t *data;
int flag = 0;
uint32_t sumCounter = 0;
volatile unsigned long int timer = 0;

// PID Variables
int ScaleFactor = 1;
int Motor = 3;
int Enable = 2;
int Position = A7;
int Integral=0;
float kP = 0.05;
float kI = 1;
float kD = 1;
int Last = 0;
int IntThresh = 1;
int minPWM = 26;
int maxPWM = 229;
uint16_t minPot;
uint16_t maxPot;
int setPoint;

uint32_t read_uint32(){
  while(Serial.available()<4);
  uint32_t out = Serial.read();
  out += (Serial.read()<<8);
  out += (Serial.read()<<16);
  out += (Serial.read()<<24);
  return out;
}
uint16_t read_uint16(){
  while(Serial.available()<2);
  uint16_t out = Serial.read();
  out += (Serial.read()<<8);
  return out;
}
void write_uint16(uint16_t val){
  Serial.write((byte)(val));
  Serial.write((byte)(val>>8));
}

void PIDcontrol(int SetPt){
   int Actual = analogRead(Position);
   int Error = SetPt - Actual;

   if (abs(Error) < IntThresh){ // prevent integral 'windup'
     Integral = Integral + Error; // accumulate the error integral
   }
   else {
   Integral=0; // zero it if out of bounds
   }
   float P = Error*kP; // calc proportional term
   float I = Integral*kI; // integral term
   float D = (Last-Actual)*kD; // derivative term
   int Drive = P ;//+ D; // Total drive = P+I+D
   Drive = Drive*ScaleFactor + (minPWM+maxPWM)/2; // scale Drive to be in the range 0-255
   if (Drive < minPWM){ // Check which direction to go.
     Drive = minPWM;
   }

   if (Drive>maxPWM) {
     Drive=maxPWM;
   }
   analogWrite (Motor,Drive); // send PWM command to motor board
  //  Serial.println(Drive);
  //  Serial.print("  -  ");
  //  Serial.println(analogRead(Position));
   Last = Actual;
}

bool Calibration(){
  int potVal = analogRead(Position);
  if(minPot>(uint16_t)potVal){
    minPot = (uint16_t)potVal;
  }
  else if (maxPot<(uint16_t)potVal){
    maxPot = (uint16_t)potVal;
  }
  if (potVal<10||potVal>1010){
    return false;
  }
  return true;
}

void runTrajectory(){
  unsigned long int oldTimer;
  for (int i =0; i<(int)dataLength; i++){
    // oldTimer = timer;
    // while (timer - oldTimer<100){
      PIDcontrol((int)data[i]);
    // }
  }
}

// void timerCallback(){
//   timer++;
// }



extern "C" int main(void)
{
  // To use Teensy 3.0 without Arduino, simply put your code here.
  // For example:
  // pinMode(13, OUTPUT);
  // Serial.begin(115200);
  // pinMode(Enable,OUTPUT);
  // digitalWrite(Enable,HIGH);


  // Serial.println(analogRead(Position));
  // setup();
  Serial.begin(115200);
  setPoint = analogRead(Position);
  pinMode(Motor, OUTPUT);
  pinMode(Enable,OUTPUT);
  analogWrite(Motor,25);
  digitalWrite(Enable,LOW);
  // Serial.begin(115200);
  // while (1) {
  // 	PIDcontrol(setPoint);
  //   delay(10);
  //   Serial.println(analogRead(Position));
  // }

  /* Header:
  1 byte: message type
  0xFF: Start running
  0x01: Send trajectory
  4-byte data length
  4-byte checksum
  */

  // Main loop
  while (true){
    flag = 0;
    action = 0;
    cSum = 0;
    dataLength = 0;
    sumCounter =0;

// Get initilization bytes
    while(flag==0){
      if(Serial.available() >= 9){
        action = Serial.read();
        dataLength = read_uint32();
        cSum = read_uint32();
        flag = 1;
      }
    }

// State Machine
    switch(action) {
        case (uint8_t)255  : // runTrajectory
          write_uint16(3);
          runTrajectory();
          break;

       case (uint8_t)10  : // startCalibration
          minPot = (uint16_t)setPoint;
          maxPot = (uint16_t)setPoint;
          bool stopCalFlag;
          bool calibrationOutOfRange;
          stopCalFlag = true;
          calibrationOutOfRange = false;
          write_uint16((uint16_t)0);
          while(stopCalFlag){
            if(Calibration()==false){
              calibrationOutOfRange = true;
            }
            if(Serial.available() >= 1){
              if((int)Serial.read()==(int)11){ // stopCalibration
                stopCalFlag = false;
              }
            }
          }
          if (calibrationOutOfRange == true){
            write_uint16((uint16_t)666);
            write_uint16((uint16_t)0);
          }
          else{
            write_uint16(minPot);
            write_uint16(maxPot);
            write_uint16((uint16_t)0);
          }
          break;

       case (uint8_t)12  : // runStaticControl
          write_uint16((uint16_t)0);
          delay(100);
          bool stopStaticControl;
          stopStaticControl = false;
          setPoint = analogRead(Position);
          setPoint = 250;/////////////////////////////////////////////////
          digitalWrite(Enable,HIGH);
          while(!stopStaticControl){
            PIDcontrol(setPoint);
            if(Serial.available() >= 1){
              if((int)Serial.read()==(int)13){ // stopStaticControl
                stopStaticControl = true;
              }
            }
          }
          digitalWrite(Enable,LOW);
          write_uint16((uint16_t)0);
          break;
       case (uint8_t)1   : // sendTrajectory
           write_uint16(dataLength);
           data = (uint32_t *)malloc(dataLength*sizeof(uint32_t));

           for (uint32_t i =0; i<dataLength; i++){
             if(Serial.available() >= 2){
               a = read_uint16();
               data[i] = a;
             }
             else{
               i=i-1;
             }
           }
           for (uint32_t i = 0; i < dataLength; i++){
             sumCounter = sumCounter + data[i];
           }
           if (sumCounter==cSum){
             a=0;
             write_uint16(a);
           }
           else{
             a = 2;
             write_uint16(a);
           }
          break;

       default :
          delay(10);
    }
  }
}
