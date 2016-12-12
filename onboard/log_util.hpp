#ifndef LOG_UTIL_HPP_
#define LOG_UTIL_HPP_

#include "slave_bridge.hpp"
#include "biped_lcm/log_msg.hpp"
#include "common/serial_channels.hpp"

using namespace biped_lcm;

class Logger{
public:
	Logger(LCMSerialSlave& lcm);

	void err(const std::string& msg);
	void warn(const std::string& msg);
	void info(const std::string& msg);

private:
	LCMSerialSlave* lcm;
	void log_publish(uint8_t log_level, const std::string& msg_string);
};

Logger::Logger(LCMSerialSlave& lcm){
	this->lcm = &lcm;
}

inline void Logger::err(const std::string& msg){
	log_publish(log_msg::ERROR, msg);
}

inline void Logger::warn(const std::string& msg){
	log_publish(log_msg::WARN, msg);
}

inline void Logger::info(const std::string& msg){
	log_publish(log_msg::INFO, msg);
}

void Logger::log_publish(uint8_t log_level, const std::string& msg_string){
	log_msg msg;
	msg.msg = msg_string;
	lcm->publish(LOG_MSG, &msg);
}

#endif