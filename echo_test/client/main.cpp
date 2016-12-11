#include "WProgram.h"


// Simple echo routine

using byte = uint8_t;
int main(){

  Serial.begin(115200);

  while(1){
    if(Serial.available()){
      byte msg = Serial.read();

      // delay(1000);

      Serial.write(msg);
    }
  }
}