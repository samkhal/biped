#include <stdio.h>
#include <map>

// #include <lcm/lcm-cpp.hpp>
// #include <SerialStream.h>

#include "bridge.hpp"
#include "blink_command.hpp"

LCMSerialBridge::LCMSerialBridge(const std::string& port, LibSerial::SerialStreamBuf::BaudRateEnum baud)
	// : serial(port){ TODO
{
	serial.Open(port);
	serial.SetBaudRate(baud);
	if(!serial.good()){
		throw std::runtime_error("Could not open serial port "+port);
	}
}

void LCMSerialBridge::add_subscriber(uint8_t channel_id, const std::string& channel_name) {
	input_channel_ids[channel_name] = channel_id;
	lcm.subscribe(channel_name, &LCMSerialBridge::pass_to_serial, this);
}

// template <typename MessageType>
// void LCMSerialBridge::publish(const std::string& channel_name, byte* buf, uint32_t len){ 
// 	MessageType msg;
// 	msg.decode(buf, 0, len);

// 	lcm.publish(channel_name, &msg);
// }

template <typename MessageType>
void LCMSerialBridge::add_publisher(uint8_t channel_id, const std::string& channel_name) {
	// ChannelDef channel(channel_name, &ChannelDef::publish_fun<MessageType>);
	ChannelDef channel;
	channel.name = channel_name;
	channel.lcm = &lcm;
	// auto k = &ChannelDef::publish_fun<int>;
	void(ChannelDef::*f)(byte*,uint32_t) = &ChannelDef::publish_fun<MessageType>;
	channel.publish = f;
	output_channels[channel_id] = channel;
}

void LCMSerialBridge::pass_to_serial(const lcm::ReceiveBuffer* rbuf, const std::string& channel_name) {
	// Write the ID
	serial << input_channel_ids[channel_name]; 
	// Write the data length
	serial << rbuf->data_size;
	//Write the data itself
	serial.write(reinterpret_cast<byte*>(rbuf->data), rbuf->data_size);
}

// Read all available bytes, or up to max_bytes. Publish any complete LCM messages
void LCMSerialBridge::process_serial(int max_bytes){
	int bytes_left = max_bytes;
	// Keep reading while bytes are available, and we're not exceeding our byte max
	while(serial.rdbuf()->in_avail() > 0 && (bytes_left > 0 || max_bytes == -1)){

		switch(read_state){
			// Locate the first byte of the next message
			case FIND_HEADER: {
				serial >> last_channel_id;

				// Is this a valid channel ID? Invalids are skipped
				if(output_channels.count(last_channel_id)){
					read_state = READ_LEN;
				}
			}

			// Read in the datalength
			case READ_LEN: {
				serial >> datalen_buf[data_buf_p];
				data_buf_p++;

				//Are we done reading the length?
				if(data_buf_p >= DATALEN_SIZE){
					data_buf_p = 0;

					uint32_t* data_len = reinterpret_cast<uint32_t*>(datalen_buf);
					data_buf = new byte[*data_len];
					read_state = READ_DATA;
				}
			}

			// Read in the actual data
			case READ_DATA: {
				serial >> data_buf[data_buf_p];
				data_buf_p++;

				// Are we done reading data?
				if(data_buf_p >= data_len){
					data_buf_p = 0;

					// Recreate the LCM Message and publish

					ChannelDef channel = output_channels[last_channel_id];
					(channel.*channel.ChannelDef::publish)(data_buf, data_len);
				}
			}

		}	
	}

}

int LCMSerialBridge::handle() {
	return lcm.handle();

}

int main(){
	LCMSerialBridge bridge("/dev/ttyACM0");
	bridge.add_subscriber(0, "BLINK_COMMAND");
	bridge.add_publisher<blink_command>(1, "BLINK_RESPONSE");
	bridge.process_serial(10);
	return 0;
}