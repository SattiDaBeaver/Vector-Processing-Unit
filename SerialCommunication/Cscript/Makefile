# Makefile

CXX := g++
CXXFLAGS := -Wall -Wextra -std=c++17

SRC := main.cpp serialport.cpp
OBJ := $(SRC:.cpp=.o)
TARGET := main

all: $(TARGET)

$(TARGET): $(OBJ)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp serialport.hpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	@if exist main.o del /q main.o
	@if exist serialport.o del /q serialport.o
	@if exist serial.exe del /q serial.exe

