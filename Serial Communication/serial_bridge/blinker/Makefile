CXX=g++
CXXFLAGS=-std=c++11 -Wall

IDIR =include
ODIR= build/obj
LDIR =lib
BDIR = build/bin
SDIR = src

LCM_TYPES_DIR = ../types
LCM_H_DIR = build/lcm

# Use pkg-config to look up flags: PKG_CONFIG_PATH must be set
LCM_CFLAGS = `pkg-config --cflags lcm`
LCM_LDFLAGS = `pkg-config --libs lcm`

CFLAGS=-I$(IDIR) -I$(LCM_H_DIR) $(LCM_CFLAGS)
LDFLAGS=-L/usr/local/lib $(LCM_LDFLAGS)

LIBS= -lserial 

_DEPS = $(wildcard $(IDIR)/*.hpp) 
_LCM_DEPS = $(wildcard $(LCM_TYPES_DIR)/*.lcm)
_LCM_DEPS2 = $(patsubst $(LCM_TYPES_DIR)/%.lcm,$(LCM_H_DIR)/%.hpp,$(_LCM_DEPS))
DEPS = $(_DEPS) $(_LCM_DEPS2)

_OBJ = $(wildcard $(SDIR)/*.cpp)
OBJ = $(patsubst $(SDIR)/%.cpp, $(ODIR)/%.o, $(_OBJ))

all: $(BDIR)/main

.SECONDARY: 

$(LCM_H_DIR)/%.hpp: $(LCM_TYPES_DIR)/%.lcm
	lcm-gen --cpp --cpp-std=c++11 --cpp-hpath=$(LCM_H_DIR) $^

$(ODIR)/%.o: $(SDIR)/%.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(CFLAGS)

$(BDIR)/main: $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS)

 .PHONY: clean

clean:
	rm -f $(BDIR)/* $(LCM_H_DIR)/*.hpp $(ODIR)/*.o *~ $(INCDIR)/*~ 
