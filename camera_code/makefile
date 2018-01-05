QUIET=@

SRC_FOLDER=src

CPPFLAGS += -std=c++11 -Wno-write-strings
CPPFLAGS += -I include
#CPPFLAGS += -O3
LDFLAGS=`pkg-config --cflags opencv --libs opencv`

.PHONY: all bin/test

all: bin/test

bin/test: $(SRC_FOLDER)/top.cpp $(SRC_FOLDER)/StereoCamera.cpp $(SRC_FOLDER)/Camera.cpp $(SRC_FOLDER)/Detector.cpp
	$(CXX) $(CPPFLAGS) $^ -o $@ $(LDFLAGS)
