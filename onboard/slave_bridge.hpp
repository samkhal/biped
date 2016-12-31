#include "fix_std.hpp"
#include "common/serial_channels.hpp"
#include <map>

#ifndef SLAVE_BRIDGE_HPP_
#define SLAVE_BRIDGE_HPP_

enum ReadState {
	FIND_HEADER,
	READ_LEN,
	READ_DATA
};

/* This class implements a similar interface to LCM proper
	for use on a microcontroller
*/
class LCMSerialSlave {
private:
	struct ChannelDef {
		ChannelID id;
		void (ChannelDef::*decoder)(byte*,uint32_t);
		void (*handler)(ChannelID, void*);

		template <typename MessageType>
		void decoder_fun(byte* buf, uint32_t len){
			auto typed_handler = reinterpret_cast<void (*)(ChannelID, MessageType*)>(handler);

			// Decode message and call handler
			MessageType msg;
			msg.decode(buf, 0, len);

			typed_handler(id, &msg);
		}
	};

	// Map of channel IDs to message-handler functions
	std::map<ChannelID, ChannelDef> input_channels;

	ReadState read_state = FIND_HEADER;
	ChannelID last_channel_id;
	uint32_t data_len;
	byte datalen_buf[sizeof(data_len)];
	uint32_t data_buf_p = 0;
	byte* data_buf; //TODO smart pointers

public:
	LCMSerialSlave();

	/** Publish an LCM Message over serial
	 * @param channel_id byte used as channel identifier
	 * @param msg the message to publish
	 *
	 * @return 0 on success, -1 on failure
	 */
	template<typename MessageType>
	int publish(ChannelID channel_id, const MessageType* msg) const;

	/** Subscribe to a serial LCM Message that will trigger a callback function
	 * @param channel_id id of channel to subscribe to
	 * @param callback function to trigger
	 *
	 * @return 0 on success, -1 on failure
	 */
	template<typename MessageType>
	int subscribe(ChannelID channel_id, void (*handler)(ChannelID, MessageType*));

	/** Handle all available serial messages. Does not block if nothing is available.
	 * @param max_bytes maximum number of bytes to read before returning.
	 * 			        -1 means process all available bytes.
	 *
	 * @return 0 on success, -1 on failure
 	 */
	int handle(int max_bytes = -1);
};

// Include implementation here for templated functions
#define SLAVE_BRIDGE_IMPL_HPP_
#include "slave_bridge_impl.hpp"
#undef SLAVE_BRIDGE_IMPL_HPP_

#endif