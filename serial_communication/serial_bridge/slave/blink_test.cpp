#include "WProgram.h"

#include "slave_bridge.hpp"
#include "blink_command.hpp"
#include "blink_count.hpp"

#include <TimerOne.h>

volatile char blink_counter = 0;
LCMSerialSlave lcm;

void blink_LED(){
	static int interrupt_counter = 0;
	static int led_state = LOW;
	interrupt_counter++;

	if (interrupt_counter >= 10){
		interrupt_counter = 0;

		led_state = 1-led_state;
		digitalWrite(LED_BUILTIN, led_state);

		if (led_state==HIGH){
			Serial.print("Blink!");
			blink_counter++;
			// blink_count msg;
			// msg.count = blink_counter;
			// lcm.publish(1, &msg);
		}
	}
}

void blink_handler(CHANNEL_ID, blink_command* msg){
	Serial.println("LCM callback triggered!");
	digitalWrite(LED_BUILTIN, HIGH);	
	
	if(msg->command == blink_command::ON){
		Timer1.initialize(100000);
		Timer1.attachInterrupt(blink_LED);
	}
}

int main(){
	delay(1000);
	Serial.begin(115200);
	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, LOW);	


	Serial.print("Starting example\n");
	// int led_state = LOW;
	// while(1){
	// 	delay(1000);
	// 	Serial.print("Test");
	// 	led_state = 1-led_state;
	// 	digitalWrite(LED_BUILTIN, led_state);
	// }
	lcm.subscribe(0, &blink_handler);

	while(1){
		if(Serial.available()){
			Serial.print("S: available bytes: ");
			Serial.println(Serial.available());
			// while(Serial.available()){
			// 	Serial.print((int) Serial.read());
			// 	Serial.print("-");
			// }
			//clear incoming
			lcm.handle();
		}
		// lcm.handle();
	}

	return 0;
}