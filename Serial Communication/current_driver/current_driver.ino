#include <EEPROM.h>

#define SIGNAL A0
#define POT A7
#define PWM 4
#define EN 1

int lower, upper;
int goal = -1;
bool follow = false;

String getCommand(){
  Serial.print("> ");
  while(!Serial.available());
  String cmd = Serial.readString();
  Serial.println(cmd);
  return cmd;
}

void setRange(){
  String cmd = getCommand();
  int val = analogRead(POT);
  int lsb = val & 0xFF;
  int msb = (val >> 8) & 0xFF;
  if(cmd == "min"){
    EEPROM.write(8, lsb);
    EEPROM.write(9, msb);
  }
  if(cmd == "max"){
    EEPROM.write(10, lsb);
    EEPROM.write(11, msb);
  }
  setup();
}

void getRange(){
  int min_lsb = EEPROM.read(8);
  int min_msb = EEPROM.read(9);
  lower = min_msb << 8 | min_lsb;
  int max_lsb = EEPROM.read(10);
  int max_msb = EEPROM.read(11);
  upper = max_msb << 8 | max_lsb;
}

void setPins(){
  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(0, OUTPUT);
  pinMode(1, OUTPUT);
  pinMode(2, OUTPUT);
  analogWrite(3, goal);
  analogWrite(4, goal);
  analogWrite(5, goal);
  digitalWrite(0, HIGH);
  digitalWrite(1, HIGH);
  digitalWrite(2, HIGH);
}

void halt(){
  digitalWrite(EN, LOW);
  digitalWrite(13, HIGH);
  while(1);
}

void setup() {
  Serial.begin(9600);
  while(!Serial);
  // put your setup code here, to run once:
  Serial.print("Ready");
  String cmd = getCommand();
  if(cmd == "cal")
    setRange();
  else if(cmd == "print")
    while(1){
      Serial.println(analogRead(POT));
      delay(150);
    }
  else if(cmd == "target")
    goal = getCommand().toInt();
  else if(cmd == "follow")
    follow = true;
  else{
    Serial.println("Unrecognized Command: " + cmd);
    setup();
  }
  getRange();
  setPins();
  Serial.println("Go!");
}

void loop() {
  if(follow){
    goal = analogRead(SIGNAL);
  }
  analogWrite(3, goal);
  analogWrite(4, goal);
  analogWrite(5, goal);
  int state = analogRead(POT);
  if(state <= lower || state >= upper)
    halt();
}
