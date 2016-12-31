CXX=g++
CXXFLAGS=-std=c++11 -Wall

IDIR = .
ODIR= build/obj
LDIR =lib
BDIR = build
SDIR = .

LCM_TYPES_DIR = ../../biped_lcm
LCM_H_DIR = build/lcm
LCM_TARGET_JAR = $(ODIR)/types.jar
# TODO: make this independent of DRAKE_DISTRO
LCM_JAR = $(DRAKE_DISTRO)/build/install/share/java/lcm.jar

# Use pkg-config to look up flags: PKG_CONFIG_PATH must be set
LCM_CFLAGS = `pkg-config --cflags lcm`
LCM_LDFLAGS = `pkg-config --libs lcm`

CFLAGS=-I$(IDIR) -I$(LCM_H_DIR) $(LCM_CFLAGS)
LDFLAGS=-L/usr/local/lib $(LCM_LDFLAGS)

LIBS= -lserial 

_DEPS = $(wildcard $(IDIR)/*.hpp) 
_LCM_DEFS = $(wildcard $(LCM_TYPES_DIR)/*.lcm)
_LCM_DEPS = $(patsubst $(LCM_TYPES_DIR)/%.lcm,$(LCM_H_DIR)/%.hpp,$(_LCM_DEFS))
DEPS = $(_DEPS) $(_LCM_DEPS)

LCM_JAVA_DEPS = $(patsubst $(LCM_TYPES_DIR)/%.lcm,lcmtypes/%.class,$(_LCM_DEFS))
LCM_JAVA_DEPS_FULL = $(patsubst %,$(ODIR)/%,$(LCM_JAVA_DEPS))

_OBJ = $(wildcard $(SDIR)/*.cpp)
OBJ = $(patsubst $(SDIR)/%.cpp, $(ODIR)/%.o, $(_OBJ))

all: $(BDIR)/main $(LCM_TARGET_JAR)

.SECONDARY: 

$(ODIR):
	mkdir -p $(ODIR)

# CPP LCM generation
$(LCM_H_DIR)/%.hpp: $(LCM_TYPES_DIR)/%.lcm
	lcm-gen --cpp --cpp-std=c++11 --cpp-hpath=$(LCM_H_DIR) $^

# Java/matlab LCM Generation
$(ODIR)/lcmtypes/%.java: $(LCM_TYPES_DIR)/%.lcm
	lcm-gen -j --jpath $(ODIR) $<

# javac must use the same java version as matlab (check in matlab: version -java)
$(ODIR)/lcmtypes/%.class: $(ODIR)/lcmtypes/%.java
	javac -source 1.7 -target 1.7 -cp $(LCM_JAR) $<

$(LCM_TARGET_JAR):  $(LCM_JAVA_DEPS_FULL) | $(ODIR)
	@echo $(LCM_JAVA_DEPS_FULL)
	cd $(ODIR) && jar cf ../../$(LCM_TARGET_JAR) $(LCM_JAVA_DEPS)

# Compile o files
$(ODIR)/%.o: $(SDIR)/%.cpp $(DEPS) | $(ODIR)
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(CFLAGS)

$(BDIR)/main: $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS)

 .PHONY: clean

clean:
	rm -rf $(BDIR) $(LCM_H_DIR)/*.hpp $(ODIR) *~ $(INCDIR)/*~ 
