#ifndef MAIN_HPP_
#define MAIN_HPP_

#include "biped_lcm/cmd_mode.hpp"
#include "slave_bridge.hpp"
#include "common/serial_channels.hpp"

// State functions
void idle();
void pos_hold();

void cmd_handler(ChannelID channel, biped_lcm::cmd_mode* msg);

void update_robot_state(){};
void publish_robot_state();
void pid_joint_position(){};
#endif