#include "WProgram.h"


// Simple echo routine

int main(){

  Serial.begin(115200);

  while(1){
    if(Serial.available()){
      char byte = Serial.read();

      delay(2000);

      Serial.write(byte+1);
    }
  }
}