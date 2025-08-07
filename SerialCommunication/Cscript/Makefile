# Makefile for Serial Communication Project

CXX := g++
CXXFLAGS := -Wall -Wextra -std=c++17

SRC := main.cpp serialport.cpp
OBJ := $(SRC:.cpp=.o)
TARGET := main
DEBUG_TARGET := main_debug

# Default target (release build)
all: $(TARGET)

# Release build (no debug output)
$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^
	@echo Release build complete: $(TARGET).exe

# Debug build (with debug output)
debug: CXXFLAGS += -DDEBUG -g
debug: TARGET := $(DEBUG_TARGET)
debug: clean $(DEBUG_TARGET)

$(DEBUG_TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^
	@echo Debug build complete: $(DEBUG_TARGET).exe

# Object file compilation
%.o: %.cpp serialport.hpp
	$(CXX) $(CXXFLAGS) -c $<

# Build and run release version
run: $(TARGET)
	@echo Running release version...
	@./$(TARGET).exe

# Build and run debug version
run-debug: debug
	@echo Running debug version...
	@./$(DEBUG_TARGET).exe

# Clean all build artifacts
clean:
	@if exist main.o del /q main.o 2>nul
	@if exist serialport.o del /q serialport.o 2>nul
	@if exist $(TARGET).exe del /q $(TARGET).exe 2>nul
	@if exist $(DEBUG_TARGET).exe del /q $(DEBUG_TARGET).exe 2>nul
	@echo Clean complete

# Show available targets
help:
	@echo Available targets:
	@echo   all        - Build release version (default)
	@echo   debug      - Build debug version with verbose output
	@echo   run        - Build and run release version
	@echo   run-debug  - Build and run debug version
	@echo   clean      - Remove all build artifacts
	@echo   help       - Show this help message

# Phony targets (not files)
.PHONY: all debug run run-debug clean help