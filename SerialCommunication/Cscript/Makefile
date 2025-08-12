# Minimal ARM64 Clang Makefile
CXX := clang++
CXXFLAGS := -Wall -std=c++17

all: main

run: main
	@echo "Running Main"
	./main

main: main.o serialport.o
	@echo "Linking all files"
	$(CXX) $(CXXFLAGS) $^ -o main

main.o: main.cpp
	@echo "Compiling the main file"
	$(CXX) $(CXXFLAGS) -c main.cpp

serialport.o: serialport.cpp
	@echo "Compiling the serial port file"
	$(CXX) $(CXXFLAGS) -c serialport.cpp

clean:
	@echo "Remove all object and executable files"
	rm *.o main
