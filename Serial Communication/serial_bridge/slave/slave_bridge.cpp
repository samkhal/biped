#include "WProgram.h"
// #include "Arduino.h"
#include "slave_bridge.hpp"


LCMSerialSlave::LCMSerialSlave(){
}

template<typename MessageType>
int LCMSerialSlave::publish(CHANNEL_ID channel_id, const MessageType* msg){
	uint32_t size = msg.getEncodedSize();

	// Allocate enough room for the header and body
	int header_size = sizeof(channel_id) + sizeof(size);
	byte* buf = new byte[header_size+size]; 

	// Store the header, then the body
	memcpy(buf, channel_id, sizeof(channel_id));
	memcpy(buf[sizeof(channel_id)], size, sizeof(size));
	msg.encode(buf, header_size, size);

	int written = Serial.write(buf, header_size+size);
	return written>0 ? 0 : -1;
}

template<typename MessageType>
int LCMSerialSlave::subscribe(CHANNEL_ID channel_id, void (*handler)(CHANNEL_ID, MessageType*)){

	auto untyped_handler = reinterpret_cast<void (CHANNEL_ID, void*)>(handler);
	ChannelDef channel;
	channel.id = channel_id;
	channel.handler = untyped_handler;
	channel.decoder = &ChannelDef::decoder_fun<MessageType>;

	input_channels[channel_id] = channel;
}

int LCMSerialSlave::handle(){

}

namespace std {
  void __throw_bad_alloc()
  {
    Serial.println("Unable to allocate memory");
  }

  void __throw_length_error( char const*e )
  {
    Serial.print("Length Error :");
    Serial.println(e);
  }
}

// void blink_handler(CHANNEL_ID, blink_command msg){
	
// }

int main(){
	LCMSerialSlave slave;
	// slave.subscribe(1, )
	return 0;
}