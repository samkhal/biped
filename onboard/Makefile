ARDUINO_LIBS = TimerThree EEPROM
MONITOR_PORT = /dev/ttyACM0
BOARD_TAG    = teensy31

# Set the build directory ourselves
ifdef BDIR_ABS
	OBJDIR=$(BDIR_ABS)/$(BOARD_TAG)
else
$(error run make at the top level)
endif

ifndef OBJDIR
    OBJDIR = build-$(BOARD_TAG)
    ifdef BOARD_SUB
        OBJDIR = build-$(BOARD_TAG)-$(BOARD_SUB)
    endif
endif



include $(ARDMK_DIR)/Teensy.mk
