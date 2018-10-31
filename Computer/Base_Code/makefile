CC:=g++
CFLAGS:= -w -O3 -s -w `pkg-config --cflags opencv4`
LDFLAGS:= `pkg-config --libs opencv4`

objectTracking: main.cpp
	$(CC) main.cpp -o objectTracking $(CFLAGS) $(LDFLAGS)
