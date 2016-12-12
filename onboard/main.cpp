#include "WProgram.h"
#include "main.hpp"
#include "slave_bridge.hpp"
#include "fix_std.hpp"

#include "common/serial_channels.hpp"
#include "log_util.hpp"

#include <TimerOne.h>

using namespace biped_lcm;

LCMSerialSlave lcm;
Logger logger(lcm);

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
	logger.info("Publishing robot state");
}

void cmd_handler(CHANNEL_ID channel, cmd_mode* msg){
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

int main(){
	Serial.begin(115200);
	lcm.subscribe(CMD_MODE, &cmd_handler);

	while(1){
		// Call the appropriate state function
		// state_table[(int)cur_state]();
		logger.info("INFO HERE");

		lcm.handle();
		delay(1000);
	}
	return 0;
}