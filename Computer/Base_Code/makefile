CC:=g++
CFLAGS:= -std=c++14 -Wall -Wextra -O3 `pkg-config --cflags opencv4`
LDFLAGS:= `pkg-config --libs opencv4`
FILES:= main.cpp selectCamera.cpp initiateArduino.cpp initiateLogFile.cpp utilities.cpp

main: $(FILES)
	$(CC) -o main $(FILES) $(CFLAGS) $(LDFLAGS)
