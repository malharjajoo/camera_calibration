QUIET=@
LDFLAGS=`pkg-config --cflags opencv --libs opencv`
SRC_FOLDER=src
WARNING_FLAGS=-Wno-write-strings
GCC_OPTS=-O0

.PHONY: all bin/test

all: bin/test

bin/test: $(SRC_FOLDER)/stereoCalibrate.cpp
	$(CXX)$(WARNING_FLAGS) $< -o $@ $(LDFLAGS)
