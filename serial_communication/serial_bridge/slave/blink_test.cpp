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
			blink_counter++;
			blink_count msg;
			msg.count = blink_counter;
			lcm.publish(1, &msg);
		}
	}
}

void blink_handler(CHANNEL_ID, blink_command* msg){
	if(msg->command == blink_command::ON){
		Timer1.initialize(100000);
		Timer1.attachInterrupt(blink_LED);
	}
}

int main(){
	lcm.subscribe(0, &blink_handler);

	while(1){
		lcm.handle();
	}

	return 0;
}