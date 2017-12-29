QUIET=@
LDFLAGS=`pkg-config --cflags opencv --libs opencv`

.PHONY: all test

all: test

test: stereoCalibrate.cpp
	$(QUIET) $(CXX) $< -o $@ $(LDFLAGS)
