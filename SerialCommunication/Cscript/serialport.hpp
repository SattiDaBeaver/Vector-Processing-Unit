#pragma once
#include <iostream>
#include <string>
#include <windows.h>

using namespace std;

class SerialPort {
private:
    HANDLE hSerial;

public:
    // Constructor
    SerialPort();

    // Open the serial port
    bool openPort(const std::string& portName, int baudRate);
    // Send data over serial port
    bool sendData(const std::string& data);
    // Read data from serial port
    string readData(int maxBytes);
    // Check if port is open and valid
    bool isOpen();
    // Close the serial port
    void closePort();
    // Destructor - automatically close port
    ~SerialPort();
};