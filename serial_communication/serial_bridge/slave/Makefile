ARDUINO_LIBS = TimerOne 
MONITOR_PORT = /dev/ttyACM0
BOARD_TAG    = teensy31

# Set the build directory ourselves
ifndef OBJDIR
    OBJDIR = build-$(BOARD_TAG)
    ifdef BOARD_SUB
        OBJDIR = build-$(BOARD_TAG)-$(BOARD_SUB)
    endif
endif

LCM_TYPES_DIR = ../types
LCM_H_DIR = $(OBJDIR)/lcm

# Add the LCM headers to the common dependencies
_LCM_DEPS = $(wildcard $(LCM_TYPES_DIR)/*.lcm)
_LCM_DEPS2 = $(patsubst $(LCM_TYPES_DIR)/%.lcm,$(LCM_H_DIR)/%.hpp,$(_LCM_DEPS))
COMMON_DEPS = $(_LCM_DEPS2)

# Add the LCM headers to the include path
CPPFLAGS := -I$(LCM_H_DIR) `pkg-config --cflags-only-I lcm`

include $(ARDMK_DIR)/Teensy.mk

# Rule for compiling the needed lcm headers
$(OBJDIR)/lcm/%.hpp: $(LCM_TYPES_DIR)/%.lcm 
	lcm-gen --cpp --cpp-std=c++11 --cpp-hpath=$(LCM_H_DIR) $^
