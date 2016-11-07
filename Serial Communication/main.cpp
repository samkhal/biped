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
int Motor = 6;
int Enable = 3;
int Position = A8;
int Integral=0;
float kP = 1;
float kI = 1;
float kD = 1;
int Last = 0;
int IntThresh = 1;
int minPWM = 26;
int maxPWM = 229;
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
  //  analogWrite (Motor,Drive); // send PWM command to motor board
   Serial.print(Drive);
   Serial.print("  -  ");
   Serial.println(analogRead(Position));
   Last = Actual;
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

void setup() {
  // Timer1.initialize(1000);         // initialize timer1, and set a 1/1000 second period
  // Timer1.attachInterrupt(timerCallback);  // attaches callback() as a timer overflow interrupt
  pinMode(13, OUTPUT);
  Serial.begin(115200);
  pinMode(Enable,OUTPUT);
  digitalWrite(Enable,HIGH);
  Serial.begin(115200);
  // while(Serial.available()==0)
  // {}
  // delay(10);
}


extern "C" int main(void)
{
  // To use Teensy 3.0 without Arduino, simply put your code here.
  // For example:
  // pinMode(13, OUTPUT);
  // Serial.begin(115200);
  // pinMode(Enable,OUTPUT);
  // digitalWrite(Enable,HIGH);
  Serial.println(analogRead(Position));
  setPoint = analogRead(Position);
  while (1) {
  	PIDcontrol(setPoint);
    delay(10);
    // Serial.println(analogRead(Position));
  }

  /* Header:
  1 byte: message type
  0xFF: Start running
  0x01: Send trajectory
  4-byte data length
  4-byte checksum
  */

  // // Main loop
  // while (true){
  //   flag = 0;
  //   action = 0;
  //   cSum = 0;
  //   dataLength = 0;
  //   sumCounter =0;
  //
  //   while(flag==0){
  //     if(Serial.available() >= 9){
  //       action = Serial.read();
  //       if (action == (uint8_t)255){
  //         write_uint16(3);
  //         runTrajectory();
  //         flag = 1;
  //         break;
  //       }
  //       dataLength = read_uint32();
  //       cSum = read_uint32();
  //       flag = 1;
  //       write_uint16(dataLength);
  //     }
  //   }
  //   if (action == (uint8_t)1){
  //     data = (uint32_t *)malloc(dataLength*sizeof(uint32_t));
  //
  //     for (uint32_t i =0; i<dataLength; i++){
  //       if(Serial.available() >= 2){
  //         a = read_uint16();
  //         data[i] = a;
  //       }
  //       else{
  //         i=i-1;
  //       }
  //     }
  //     for (uint32_t i = 0; i < dataLength; i++){
  //       sumCounter = sumCounter + data[i];
  //     }
  //     if (sumCounter==cSum){
  //       a=0;
  //       write_uint16(a);
  //     }
  //     else{
  //       a = 2;
  //       write_uint16(a);
  //     }
  //   }
  // }
}
