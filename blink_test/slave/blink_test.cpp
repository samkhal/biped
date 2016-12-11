#include "WProgram.h"

#include "slave_bridge.hpp"
#include "blink_command.hpp"
#include "blink_count.hpp"

#include <TimerOne.h>

/** Simple client. Waits for LCM blink command message, 
 * the publishes a blink count every second roughly.
 */

LCMSerialSlave lcm;

// Simple LED blinker
void blink_LED(){
	static int interrupt_counter = 0;
	static int led_state = LOW;
	static int blink_counter = 0;
	interrupt_counter++;

	if (interrupt_counter >= 10){ //blink every 10 interrupts
		interrupt_counter = 0;

		led_state = 1-led_state;
		digitalWrite(LED_BUILTIN, led_state);

		if (led_state==HIGH){
			//Publish LCM
			blink_counter++;
			blink_count msg;
			msg.count = blink_counter;
			lcm.publish(1, &msg);
		}
	}
}

// LCM callback. Starts blinking if it's an ON command
void blink_handler(CHANNEL_ID id, blink_command* msg){
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

	lcm.subscribe(0, &blink_handler);

	while(1){
		lcm.handle();
	}

	return 0;
}