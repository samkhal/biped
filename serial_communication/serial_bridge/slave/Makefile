ARDUINO_LIBS = TimerOne 
BOARD_TAG    = teensy31

LCM_TYPES_DIR = ../types
LCM_H_DIR = build-teensy31/lcm
# USER_LIB_PATH := $(realpath ./$(LCM_H_DIR))
# USER_LIB_PATH = $(realpath .)/build-teensy31/lcm
_LCM_DEPS = $(wildcard $(LCM_TYPES_DIR)/*.lcm)
_LCM_DEPS2 = $(patsubst $(LCM_TYPES_DIR)/%.lcm,$(LCM_H_DIR)/%.hpp,$(_LCM_DEPS))
COMMON_DEPS = $(_LCM_DEPS2)

CPPFLAGS := -I$(LCM_H_DIR) `pkg-config --cflags-only-I lcm`

include $(ARDMK_DIR)/Teensy.mk

# BOARD_TAG = uno
# include $(ARDMK_DIR)/Arduino.mk
$(OBJDIR)/lcm/%.hpp: $(LCM_TYPES_DIR)/%.lcm 
	lcm-gen --cpp --cpp-std=c++11 --cpp-hpath=$(LCM_H_DIR) $^
