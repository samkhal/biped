#include <map>

#ifndef __SLAVE_BRIDGE_H__
#define __SLAVE_BRIDGE_H__

// using CHANNEL_ID = uint8_t;
typedef uint8_t CHANNEL_ID;

// typedef char byte;

/* This class implements a similar interface to LCM proper
	for use on a microcontroller
*/
// using namespace std;
class LCMSerialSlave {
private:
	struct ChannelDef {
		CHANNEL_ID id;
		void (ChannelDef::*decoder)(byte*,uint32_t);
		void (ChannelDef::*handler)(CHANNEL_ID, void*);

		template <typename MessageType>
		void decoder_fun(byte* buf, uint32_t len){
			auto typed_handler = reinterpret_cast<void (*)(CHANNEL_ID, MessageType*)>(handler);

			// Decode message and call handler
			MessageType msg;
			msg.decode(buf, 0, len);

			typed_handler(id, &msg);
		}
	};

	// Map of channel IDs to message-handler functions
	std::map<CHANNEL_ID, ChannelDef> input_channels;

public:
	LCMSerialSlave();


	/** Publish an LCM Message over serial
	 * @param channel_id byte used as channel identifier
	 * @param msg the message to publish
	 *
	 * @return 0 on success, -1 on failure
	 */
	template<typename MessageType>
	int publish(CHANNEL_ID channel_id, const MessageType* msg);

	/** Subscribe to a serial LCM Message that will trigger a callback function
	 * @param channel_id id of channel to subscribe to
	 * @param callback function to trigger
	 *
	 * @return 0 on success, -1 on failure
	 */
	template<typename MessageType>
	int subscribe(CHANNEL_ID channel_id, void (*handler)(CHANNEL_ID, MessageType*));

	/** Handle all available serial messages. Does not block if none are available.
	 *
	 * @return 0 on success, -1 on failure
 	 */
	int handle();
};

#endif