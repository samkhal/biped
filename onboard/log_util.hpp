#ifndef LOG_UTIL_HPP_
#define LOG_UTIL_HPP_

#include <iostream>
#include <sstream>

#include "slave_bridge.hpp"
#include "biped_lcm/log_msg.hpp"
#include "common/serial_channels.hpp"

using namespace biped_lcm;

class Logger : public std::ostream{
private:
	class LogBuf : public std::stringbuf{
	private:
		const LCMSerialSlave& lcm;
		const uint8_t log_level;	
	public:
		LogBuf(const LCMSerialSlave& lcm, uint8_t log_level):
			lcm(lcm), log_level(log_level){}
		~LogBuf(){
			pubsync();
		}

		// sync() is called on std::flush (or std::endl)
		// publish the current string on lcm
		int sync(){
			log_msg msg;
			msg.msg = str();
			str(""); // Clear current string
			msg.log_level = log_level;
			return lcm.publish(ChannelID::LOG_MSG, &msg);
		}
	};
public:
	Logger(const LCMSerialSlave& lcm, uint8_t log_level):
		std::ostream(new LogBuf(lcm, log_level)){}
	~Logger(){ 
		delete rdbuf();
	}
};


#endif