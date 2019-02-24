CC:=g++
CFLAGS:= -Wall -Wextra -O3 `pkg-config --cflags opencv4`
LDFLAGS:= `pkg-config --libs opencv4`

objectTracking: main.cpp
	$(CC) main.cpp -o main $(CFLAGS) $(LDFLAGS)
