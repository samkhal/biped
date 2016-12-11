# List all independent nodes, separated with escaped newlines
NODES = \
serial_bridge 

# Code running onboard
ONBOARD = onboard_tmp

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

LCM_DEFS = $(wildcard $(LCM_TYPES_DIR)/*.lcm)

default: all

# -----------Build LCM CPP:
LCM_B_DIR = $(BUILD_DIR)/$(LCM_TYPES_DIR)
export LCM_BDIR_ABS=$(MKFILE_DIR)$(LCM_B_DIR)

LCM_HEADERS = $(patsubst $(LCM_TYPES_DIR)/%.lcm, $(LCM_B_DIR)/%.hpp, $(LCM_DEFS))
$(LCM_B_DIR)/%.hpp: $(LCM_TYPES_DIR)/%.lcm
	lcm-gen --cpp --cpp-std=c++11 --cpp-hpath=$(LCM_B_DIR) $^

#----------- Build LCM Java: 
LCM_JAVA_DIR = $(LCM_B_DIR)/$(LCM_PACKAGE_NAME)
# Build .java and .class files
$(LCM_JAVA_DIR)/%.class: $(LCM_TYPES_DIR)/%.lcm
	mkdir -p $(LCM_B_DIR)
	lcm-gen -j --jpath $(LCM_B_DIR) $<
	javac -source $(MATLAB_JAVA) -target $(MATLAB_JAVA) -cp $(LCM_JAR) $(LCM_JAVA_DIR)/$*.java

# Build .jar file
LCM_JAR_DEPS = $(patsubst $(LCM_TYPES_DIR)/%.lcm, $(LCM_PACKAGE_NAME)/%.class, $(LCM_DEFS))
$(LCM_B_DIR)/$(LCM_TARGET_JAR): $(addprefix $(LCM_B_DIR)/,$(LCM_JAR_DEPS))
	cd $(LCM_B_DIR) && jar cf $(LCM_TARGET_JAR) $(LCM_JAR_DEPS)

#------------Build onboard
export OBJDIR = $(realpath $(BUILD_DIR))/$(BOARD-TAG)
# Add the LCM headers to the include path
export CPPFLAGS = -I$(realpath $(LCM_B_DIR)) `pkg-config --cflags-only-I lcm`

onboard: $(LCM_HEADERS) 
	# @echo "Entering onboard submake"
	# @echo $(CPPFLAGS)
	# @echo $(MKFILE_DIR)
	# set makelevel to not suppress config output
	export MAKELEVEL=0;\
	$(MAKE) -C $(ONBOARD)

#------------Build nodes
NODE_TARGETS = $(addprefix $(BUILD_DIR)/,$(addsuffix /$(NODE_TARGET_NAME), $(NODES)))

# Use pkg-config to look up flags: PKG_CONFIG_PATH must be set
LCM_CFLAGS = `pkg-config --cflags lcm`
LCM_LDFLAGS = `pkg-config --libs lcm`

INCLUDE=$(addprefix -I, $(NODE_TARGETS))
CFLAGS=-I$(INCLUDE) -I$(LCM_B_DIR) $(LCM_CFLAGS)
LDFLAGS=-L/usr/local/lib $(LCM_LDFLAGS)

SOURCES = $(foreach t, $(NODE_TARGETS), $(wildcard t/*.cpp))
OBJECTS = $(addprefix $(BUILD_DIR)/, $(SOURCES:%.cpp=%.o))

$(NODE_TARGETS): $(BUILD_DIR)/%/$(NODE_TARGET_NAME):
	# @echo TARGET $@
	# @echo MATCH $*
	# @echo $(NODE_TARGETS)
	@echo Building $*

	export BDIR_ABS=$(MKFILE_DIR)$(BUILD_DIR)/$*; \
	export TARGET=$(notdir $@); \
	$(MAKE) -C $*

nodes: $(NODE_TARGETS)

#------------Generic targets
all: $(LCM_HEADERS) $(LCM_B_DIR)/$(LCM_TARGET_JAR) onboard nodes
	@echo $(LCM_HEADERS)

clean:
	rm -rf $(BUILD_DIR)


