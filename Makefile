# List all independent nodes, separated with escaped newlines
NODES = \
serial_bridge 

# Code running onboard
ONBOARD = onboard

LCM_TYPES_DIR = biped_lcm
LCM_PACKAGE_NAME = biped_lcm
LCM_TARGET_JAR = biped_lcm.jar

MATLAB_JAVA = 1.7

# TODO: make this independent of DRAKE_DISTRO
LCM_JAR = $(DRAKE_DISTRO)/build/install/share/java/lcm.jar

#----------------------------------------------

export CXX=g++
export CXXFLAGS=-std=c++11 -Wall

BUILD_DIR = build

NODE_TARGET_NAME = main

#path to this makefile, i.e. to the top level biped dir
MKFILE_DIR := $(dir $(abspath $(lastword $(MAKEFILE_LIST))))
export BDIR_ABS=$(realpath $(BUILD_DIR))

#----------- LCM vars
LCM_DEFS = $(wildcard $(LCM_TYPES_DIR)/*.lcm)
LCM_B_DIR = $(BUILD_DIR)/$(LCM_TYPES_DIR)
export LCM_BDIR_ABS=$(MKFILE_DIR)$(LCM_B_DIR)

LCM_HEADERS = $(patsubst $(LCM_TYPES_DIR)/%.lcm, $(LCM_B_DIR)/%.hpp, $(LCM_DEFS))
LCM_JAVA_DIR = $(LCM_B_DIR)/$(LCM_PACKAGE_NAME)
LCM_JAR_DEPS = $(patsubst $(LCM_TYPES_DIR)/%.lcm, $(LCM_PACKAGE_NAME)/%.class, $(LCM_DEFS))

#---------- Other misc vars
# Used by onboard compilation
# Add the LCM headers to the include path
export CPPFLAGS=-I$(realpath $(LCM_B_DIR)) `pkg-config --cflags-only-I lcm`

NODE_TARGETS = $(addprefix $(BUILD_DIR)/,$(addsuffix /$(NODE_TARGET_NAME), $(NODES)))

default: all

#Build LCM CPP:

$(LCM_B_DIR)/%.hpp: $(LCM_TYPES_DIR)/%.lcm
	lcm-gen --cpp --cpp-std=c++11 --cpp-hpath=$(LCM_B_DIR) $^

# Build LCM Java: 
# Build .java and .class files
$(LCM_JAVA_DIR)/%.class: $(LCM_TYPES_DIR)/%.lcm
	lcm-gen -j --jpath $(LCM_B_DIR) $<
	javac -source $(MATLAB_JAVA) -target $(MATLAB_JAVA) -cp $(LCM_JAR) $(LCM_JAVA_DIR)/$*.java

# Build .jar file
$(LCM_B_DIR)/$(LCM_TARGET_JAR): $(addprefix $(LCM_B_DIR)/,$(LCM_JAR_DEPS))
	cd $(LCM_B_DIR) && jar cf $(LCM_TARGET_JAR) $(LCM_JAR_DEPS)

#Build onboard
onboard: $(LCM_HEADERS) 
	# set makelevel to not suppress config output
	export MAKELEVEL=0;\
	$(MAKE) -C $(ONBOARD)

upload: $(LCM_HEADERS)
	export MAKELEVEL=0;\
	$(MAKE) upload -C $(ONBOARD)

#Build nodes
$(NODE_TARGETS): $(BUILD_DIR)/%/$(NODE_TARGET_NAME):
	@echo Building $*

	export BDIR_ABS=$(MKFILE_DIR)$(BUILD_DIR)/$*; \
	export TARGET=$(notdir $@); \
	$(MAKE) -C $*

#------------Generic targets
nodes: $(NODE_TARGETS)

lcm-cpp: $(LCM_HEADERS)

lcm-java: $(LCM_B_DIR)/$(LCM_TARGET_JAR)

all: lcm-cpp lcm-java onboard nodes

clean:
	rm -rf $(BUILD_DIR)


