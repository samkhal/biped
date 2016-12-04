// Implementation of slave_bridge.hpp

#ifndef __SLAVE_BRIDGE_IMPL__
#error "Don't include this file directly"
#endif

LCMSerialSlave::LCMSerialSlave(){
}

template<typename MessageType>
int LCMSerialSlave::publish(CHANNEL_ID channel_id, const MessageType* msg){
	uint32_t size = msg->getEncodedSize();

	// Allocate enough room for the header and body
	int header_size = sizeof(channel_id) + sizeof(size);
	byte* buf = new byte[header_size+size]; 

	// Store the header, then the body
	memcpy(buf, &channel_id, sizeof(channel_id));
	memcpy(&buf[sizeof(channel_id)], &size, sizeof(size));
	msg->encode(buf, header_size, size);

	int written = Serial.write(buf, header_size+size);
	Serial.send_now(); //Flush message

	return written>0 ? 0 : -1;
}

template<typename MessageType>
int LCMSerialSlave::subscribe(CHANNEL_ID channel_id, void (*handler)(CHANNEL_ID, MessageType*)){

	auto untyped_handler = reinterpret_cast<void (*)(CHANNEL_ID, void*)>(handler);
	ChannelDef channel;
	channel.id = channel_id;
	channel.handler = untyped_handler;
	channel.decoder = &ChannelDef::decoder_fun<MessageType>;

	input_channels[channel_id] = channel;
	return 0; //TODO
}

int LCMSerialSlave::handle(int max_bytes){
	int bytes_left = max_bytes;

	while(Serial.available() && (bytes_left > 0 || max_bytes == -1)){
		switch(read_state){
			// Locate the first byte of the next message
			case FIND_HEADER: {
				last_channel_id = Serial.read();

				// Is this a valid channel ID? Invalids are skipped
				if(input_channels.count(last_channel_id)){
					read_state = READ_LEN;
				}
			}

			// Read in the datalength
			case READ_LEN: {
				datalen_buf[data_buf_p] = Serial.read();
				data_buf_p++;

				//Are we done reading the length?
				if(data_buf_p >= sizeof(data_len)){
					data_buf_p = 0;

					uint32_t* data_len = reinterpret_cast<uint32_t*>(datalen_buf);
					data_buf = new byte[*data_len];
					read_state = READ_DATA;
				}
			}

			// Read in the actual data
			case READ_DATA: {
				data_buf[data_buf_p] = Serial.read();
				data_buf_p++;

				// Are we done reading data?
				if(data_buf_p >= data_len){
					data_buf_p = 0;

					// Trigger the callback for this message
					ChannelDef channel = input_channels[last_channel_id];
					(channel.*channel.ChannelDef::decoder)(data_buf, data_len);
				}
			}
		}
	}

	return 0; //TODO
}




