# Minimal ARM64 Clang Makefile
CXX := clang++
CXXFLAGS := -Wall -std=c++17
DEBUGFLAGS := -DDEBUG -g

all: main

debug: CXXFLAGS += $(DEBUGFLAGS)
debug: clean main
	@echo "Debug build complete"

debug-run: debug
	@echo "Running Debug Main"
	./main

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

# Show available targets
help:
	@echo "Available targets:"
	@echo   "all        - Build release version - default"
	@echo   "debug      - Build debug version with verbose output"
	@echo   "debug-run  - Build and run debug version with verbose output"
	@echo   "run        - Build (if needed) and run the last compiled version (debug or release)"
	@echo   "clean      - Remove all build artifacts"
	@echo   "help       - Show this help message"
