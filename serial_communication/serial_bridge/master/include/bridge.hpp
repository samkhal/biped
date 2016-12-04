#include <stdio.h>
#include <map>

#include <SerialStream.h>
#include <lcm/lcm-cpp.hpp>

#ifndef __MASTER_BRIDGE_H__
#define __MASTER_BRIDGE_H__


using byte = char;

enum ReadState {
	FIND_HEADER,
	READ_LEN,
	READ_DATA
};

class LCMSerialBridge {
private:
	struct ChannelDef {
		std::string name;
		lcm::LCM* lcm;
		void (ChannelDef::*publish)(byte*,uint32_t);

		ChannelDef(){};
		ChannelDef(const std::string& channel_name, void(ChannelDef::*publish_)(byte*,uint32_t)) :
			name(channel_name), publish(publish_){};

		template <typename MessageType>
		void publish_fun(byte* buf, uint32_t len){
			MessageType msg;
			msg.decode(buf, 0, len);
			lcm->publish(name, &msg);
		}

	};

	lcm::LCM lcm;
	LibSerial::SerialStream serial;

	std::map<const std::string, uint8_t> input_channel_ids;
	std::map<uint8_t, ChannelDef> output_channels;

	ReadState read_state = FIND_HEADER;
	uint8_t last_channel_id;
	uint32_t data_len;
	byte datalen_buf[sizeof(data_len)];
	uint32_t data_buf_p = 0;
	byte* data_buf;

public:
	/** Initialize serial connection
	 * @param port name of the serial port
	 * @param baud baudrate
	 */
	LCMSerialBridge(const std::string& port, 
		LibSerial::SerialStreamBuf::BaudRateEnum = LibSerial::SerialStreamBuf::BAUD_115200);

	/** Add a channel that the bridge subscribes to and sends across
	 * the serial connection 
	 * @param channel_id the byte used as a channel identifier
	 * @param channel_name name of lcm channel
	 */
	void add_subscriber(uint8_t channel_id, const std::string& channel_name);

	/** Add a channel that the bridge reads from serial and publishes
	 * out across LCM
	 * @param channel_id byte used as channel identifier
	 * @param channel_name name of lcm channel
	 */
	template <typename MessageType>
	void add_publisher(uint8_t channel_id, const std::string& channel_name);


	void pass_to_serial(const lcm::ReceiveBuffer* rbuf, const std::string& channel_name);
	void process_serial(int max_bytes = -1);
	int handle();
};

#endif