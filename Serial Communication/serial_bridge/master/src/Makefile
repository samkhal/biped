CXX=g++
CXXFLAGS=-std=c++11 -Wall

IDIR =../include
ODIR=obj
LDIR =../lib
LCM_TYPES_DIR=../../types
LCM_H_DIR=lcm

# Use pkg-config to look up flags: PKG_CONFIG_PATH must be set
LCM_CFLAGS = `pkg-config --cflags lcm`
LCM_LDFLAGS = `pkg-config --libs lcm`

CFLAGS=-I$(IDIR) $(LCM_CFLAGS)
LDFLAGS=-L/usr/local/lib $(LCM_LDFLAGS)

LIBS= -lserial 

_DEPS = bridge.hpp
DEPS = $(patsubst %,$(IDIR)/%,$(_DEPS))

_OBJ = bridge.o
OBJ = $(patsubst %,$(ODIR)/%,$(_OBJ))

all: lcm/blink_command.hpp main

lcm/%.hpp: $(LCM_TYPES_DIR)/%.lcm
	lcm-gen -x --cpp-std=c++11 --cpp-hpath=$(LCM_H_DIR) $^

$(ODIR)/%.o: %.cpp $(DEPS)
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(CFLAGS)

main: $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS)

.PHONY: clean

clean:
	rm -f $(ODIR)/*.o *~ $(INCDIR)/*~ 


