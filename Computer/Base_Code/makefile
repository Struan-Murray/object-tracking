CC:=g++
CFLAGS:= -std=c++17 -Wall -Wextra -pthread -lstdc++fs -O3 `pkg-config --cflags opencv4`
LDFLAGS:= `pkg-config --libs opencv4`
FILES:= main.cpp utilities.cpp

main: $(FILES)
	$(CC) -o main $(FILES) $(CFLAGS) $(LDFLAGS)
