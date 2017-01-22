// Common include for info shared by teensy and other processes
#ifndef SERIAL_CHANNELS_HPP_
#define SERIAL_CHANNELS_HPP_

// IDs for serial channels
enum class ChannelID : uint8_t {CMD_IN, CMD_RESPONSE, LOG_MSG, LIVE_IN, LIVE_OUT};

#endif
