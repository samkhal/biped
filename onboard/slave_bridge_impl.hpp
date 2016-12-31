// Implementation of slave_bridge.hpp

#ifndef SLAVE_BRIDGE_IMPL_HPP_
#error "Don't include this file directly"
#endif

LCMSerialSlave::LCMSerialSlave(){
}

template<typename MessageType>
int LCMSerialSlave::publish(ChannelID channel_id, const MessageType* msg) const{
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

	delete buf;

	return written>0 ? 0 : -1;
}

template<typename MessageType>
int LCMSerialSlave::subscribe(ChannelID channel_id, void (*handler)(ChannelID, MessageType*)){

	auto untyped_handler = reinterpret_cast<void (*)(ChannelID, void*)>(handler);
	ChannelDef channel;
	channel.id = channel_id;
	channel.handler = untyped_handler;
	channel.decoder = &ChannelDef::decoder_fun<MessageType>;

	input_channels[channel_id] = channel;
	return 0; //TODO proper return values
}

int LCMSerialSlave::handle(int max_bytes){
	int bytes_left = max_bytes;

	while(Serial.available() && (bytes_left > 0 || max_bytes == -1)){
		byte next_byte = Serial.read();
		bytes_left--;

		switch(read_state){
			// Locate the first byte of the next message
			case FIND_HEADER: {
				// Is this a valid channel ID? Invalids are skipped
				ChannelID chan_id = static_cast<ChannelID>(next_byte);
				if(input_channels.count(chan_id)){
					last_channel_id = chan_id;
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

					uint32_t* data_len_p = reinterpret_cast<uint32_t*>(datalen_buf);
					data_len = *data_len_p; //TODO cleanup this
					data_buf = new byte[data_len];
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

					// Trigger the callback for this message
					ChannelDef channel = input_channels[last_channel_id];
					(channel.*channel.ChannelDef::decoder)(data_buf, data_len);

					delete data_buf;

					read_state = FIND_HEADER;
				}
				break;
			}

			default: {
				break;
			}
		}
	}

	return 0; //TODO proper return values
}




