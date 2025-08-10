#pragma once
#include <iostream>
#include <string>
#include <windows.h>
#include <vector>
#include <stdint.h>

using namespace std;

class SerialPort {
private:
    HANDLE hSerial;

public:
    // Constructor
    SerialPort();

    // Open the serial port
    bool openPort(const string& portName, int baudRate);
    // Send data over serial port
    bool sendData(const string& data);
    bool sendData(const uint8_t* data, size_t size);
    bool sendData(const vector<uint8_t>& data);

    // Read data from serial port
    string readData(int maxBytes);
    // Check if port is open and valid
    bool isOpen();
    // Close the serial port
    void closePort();
    // Destructor - automatically close port
    ~SerialPort();
};