#Common makefile for nodes written in cpp

ifndef BDIR_ABS
$(error run make at the top level)
endif

BDIR = $(BDIR_ABS)

ifndef CXXFLAGS
CXX=g++
CXXFLAGS=-std=c++11 -Wall
endif

# include src directories
ifndef IDIR
IDIR = .
endif

ifndef SDIR
SDIR = .
endif

LCM_CFLAGS = `pkg-config --cflags lcm`
LCM_LDFLAGS = `pkg-config --libs lcm`

CFLAGS=-I$(IDIR) -I$(LCM_BDIR_ABS) $(LCM_CFLAGS)
LDFLAGS=-L/usr/local/lib $(LCM_LDFLAGS)

DEPS = $(wildcard $(IDIR)/*.hpp) 

_OBJ = $(wildcard $(SDIR)/*.cpp)
OBJ = $(patsubst $(SDIR)/%.cpp, $(BDIR)/%.o, $(_OBJ))

all: $(BDIR)/$(TARGET)

$(BDIR):
	mkdir -p $(BDIR)

$(BDIR)/%.o: $(SDIR)/%.cpp $(DEPS) | $(BDIR)
	@echo $(BDIR)
	echo STOP
	$(CXX) $(CXXFLAGS) -c -o $@ $< $(CFLAGS)

$(BDIR)/$(TARGET): $(OBJ)
	@echo WHAT
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS) $(LIBS)
