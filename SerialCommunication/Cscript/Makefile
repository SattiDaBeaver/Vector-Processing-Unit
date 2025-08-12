# Minimal x86-64 and ARM64 GCC/Clang Makefile

ifeq ($(wildcard .arch),)
# If .arch file does not exist, default ARCH to empty (or a default)
	ARCH := 
else
	ARCH := $(shell cat .arch)
endif

# Map the numeric code to arch name
ifeq ($(ARCH),9)
    CXX := clang++
	ARCH_NAME := ARM64
else ifeq ($(ARCH),12)
    CXX := clang++
	ARCH_NAME := ARM64EC
else ifeq ($(ARCH),6)
    CXX := g++
	ARCH_NAME := AMD64
else
    CXX := g++  # fallback default
	ARCH_NAME := Unknown
endif

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
	@echo "Removing all object and executable files"
	rm -f *.o main

info: 
	@echo "Processor Architecture: "$(ARCH_NAME)

update-arch:
	@echo "Updating architecture info..."
	@powershell -NoProfile -Command "(Get-CimInstance Win32_Processor).Architecture" > .arch


# Show available targets
help:
	@echo "Available targets:"
	@echo   "all        - Build release version - default"
	@echo   "debug      - Build debug version with verbose output"
	@echo   "debug-run  - Build and run debug version with verbose output"
	@echo   "run        - Build (if needed) and run the last compiled version (debug or release)"
	@echo   "clean      - Remove all build artifacts"
	@echo   "info       - Processor Architecture Information"
	@echo   "update-arch - Update cached architecture information"
	@echo   "help       - Show this help message"

.PHONY: all clean run debug debug-run help info update-arch
