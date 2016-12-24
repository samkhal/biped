#include "WProgram.h"
#include "main.hpp"
#include "slave_bridge.hpp"
#include "fix_std.hpp"

#include "common/serial_channels.hpp"
#include "biped_lcm/log_msg.hpp"
#include "log_util.hpp"

#include <TimerOne.h>

using namespace biped_lcm;

LCMSerialSlave lcm;
Logger loginfo(lcm, log_msg::INFO);
Logger logwarn(lcm, log_msg::WARN);
Logger logerr(lcm, log_msg::ERROR);

// States and state table
enum class State : uint8_t {IDLE=0, POS_HOLD};
void (*state_table[])() = {idle, pos_hold};

State cur_state = State::IDLE;

// State functions
void idle(){
	update_robot_state();
	publish_robot_state();
}

void pos_hold(){
	update_robot_state();
	publish_robot_state();

	pid_joint_position();
}

void publish_robot_state(){
	static int pub_count = 0;
	loginfo << "Publishing robot state " << pub_count << std::flush;
	pub_count++;
}

void cmd_handler(ChannelID channel, cmd_mode* msg){
	switch(msg->cmd) {
	case cmd_mode::IDLE:
		cur_state = State::IDLE;
		break;
	case cmd_mode::HOLD_POSITION:
		cur_state = State::POS_HOLD;
		break;
	default:
		break;
	}
}

// Simple debug routines
void toggle_LED(int skips=0){
	static bool first_run = true;
	static int led_state = LOW;
	static int skip_count = skips;

	if(first_run){
		pinMode(LED_BUILTIN, OUTPUT);
		first_run = false;
	}else if(skip_count>0){
		skip_count--;
	}else{
		led_state = 1-led_state;
		skip_count = skips;
	}
	digitalWrite(LED_BUILTIN, led_state);
}

int main(){
	Serial.begin(115200);
	lcm.subscribe(ChannelID::CMD_MODE, &cmd_handler);

	while(1){
		// Call the appropriate state function
		state_table[(int)cur_state]();
		toggle_LED(100);

		lcm.handle();
		// delay(10);
	}
	return 0;
}