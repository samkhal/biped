#include <stdio.h>
#include <map>
#include <unistd.h>
#include <iostream>

#include "bridge.hpp"
#include "blink_command.hpp"

//TODO move to impl file
#define DEBUG

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
	#ifdef DEBUG
	std::cout << "Subscribing to " << channel_name << std::endl;
	#endif
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
	#ifdef DEBUG
	std::cout << "passing message to serial" << std::endl;
	std::cout << "Channel: "<<(int)input_channel_ids[channel_name]<< " Size: " << rbuf->data_size<<std::endl;
	byte* bytes = (byte*)&rbuf->data_size;
	for(int i=0;i<sizeof(rbuf->data_size);i++){
		std::cout << (uint) bytes[i] << "-";
		// serial << bytes[i];
	}
	std::cout<<std::endl;
	#endif
	// Write the ID
	serial << input_channel_ids[channel_name]; 
	// Write the data length
	// serial << rbuf->data_size; //TODO why doesn't this work?
	serial.write((byte*)(&rbuf->data_size), sizeof(rbuf->data_size)); //TODO cast properly
	//Write the data itself
	serial.write(reinterpret_cast<byte*>(rbuf->data), rbuf->data_size);
}

// Read all available bytes, or up to max_bytes. Publish any complete LCM messages
void LCMSerialBridge::process_serial(int max_bytes){
	int bytes_left = max_bytes;
	// Keep reading while bytes are available, and we're not exceeding our byte max
	while(serial.rdbuf()->in_avail() > 0 && (bytes_left > 0 || max_bytes == -1)){

		byte next_byte;
		serial >> next_byte;
		bytes_left--;
		switch(read_state){
			// Locate the first byte of the next message
			case FIND_HEADER: {
				// Is this a valid channel ID? Invalids are skipped
				if(output_channels.count(next_byte)){
					last_channel_id = next_byte;
					read_state = READ_LEN;
				}
				break;
			}

			// Read in the datalength
			case READ_LEN: {
				datalen_buf[data_buf_p] = next_byte;
				data_buf_p++;

				//Are we done reading the length?
				if(data_buf_p >= sizeof(data_len)){
					data_buf_p = 0;

					uint32_t* data_len = reinterpret_cast<uint32_t*>(datalen_buf);
					data_buf = new byte[*data_len];
					read_state = READ_DATA;
				}
				break;
			}

			// Read in the actual data
			case READ_DATA: {
				data_buf[data_buf_p] = next_byte;
				data_buf_p++;

				// Are we done reading data?
				if(data_buf_p >= data_len){
					data_buf_p = 0;

					// Recreate the LCM Message and publish

					ChannelDef channel = output_channels[last_channel_id];
					(channel.*channel.ChannelDef::publish)(data_buf, data_len);
					read_state = FIND_HEADER;
				}
				break;
			}

			default:
				break;
		}	
	}

}

int LCMSerialBridge::handle(int timeout_ms) {
	return lcm.handleTimeout(timeout_ms);

}

void command_listener(const lcm::ReceiveBuffer* rbuf, 
					const std::string& channel, 
					const blink_command* msg, void* context){
	std::cout << "Received command!" << std::endl;
} 

int main(){
	// lcm::LCM lcm;
	// if(!lcm.good())
	// 	return 1;

	// lcm.subscribeFunction("BLINK_COMMAND", &command_listener, (void*)nullptr);
	// while(0 == lcm.handle());
	// return 0;


	LCMSerialBridge bridge("/dev/ttyACM0");
	std::cout << "Opened port" << std::endl;
	bridge.add_subscriber(0, "BLINK_COMMAND");
	bridge.add_publisher<blink_command>(1, "BLINK_COUNT");

	while(1){
		bridge.handle(1);
		// std::cout<<"m"<<std::endl;
		if(bridge.serial.rdbuf()->in_avail() > 0){
			char c;
			bridge.serial >> c;
			std::cout << c <<std::flush;
		}


		// std::cout << "m" << std::endl;
		// bridge.process_serial(10);
		usleep(1000); //1 ms
	}
	return 0;
}