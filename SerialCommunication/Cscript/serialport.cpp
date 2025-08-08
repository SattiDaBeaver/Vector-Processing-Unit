#include <iostream>
#include "serialport.hpp"

// Constructor
SerialPort::SerialPort() {
    hSerial = INVALID_HANDLE_VALUE;
}

// Destructor - automatically close port
SerialPort::~SerialPort() {
    closePort();
}

// Open the serial port
bool SerialPort::openPort(const string& portName, int baudRate) {
    // For COM ports > 9, Windows requires special format: \\.\COM10
    string actualPortName = portName;
    if (portName.find("COM") == 0) {
        // Extract COM number
        string comNumber = portName.substr(3);
        int portNum = stoi(comNumber);
        if (portNum > 9) {
            actualPortName = "\\\\.\\" + portName;
        }
    }

#ifdef DEBUG
    cout << "Attempting to open: " << actualPortName << endl;
#endif

    // Open the serial port
    // Use CreateFileA for ANSI string, CreateFileW for wide strings
    hSerial = CreateFileA(actualPortName.c_str(),
                         GENERIC_READ | GENERIC_WRITE,  // Read and write access
                         0,                             // No sharing
                         NULL,                          // Default security attributes
                         OPEN_EXISTING,                 // Port must exist
                         FILE_ATTRIBUTE_NORMAL,         // Normal file attributes
                         NULL);                         // No template file

    if (hSerial == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
#ifdef DEBUG
        cerr << "Error opening port " << actualPortName << " (Error code: " << error << ")" << endl;
        
        // Common error explanations
        switch(error) {
            case ERROR_FILE_NOT_FOUND:
                cerr << "  -> Port does not exist. Check Device Manager for correct COM port." << endl;
                break;
            case ERROR_ACCESS_DENIED:
                cerr << "  -> Port is already in use by another application." << endl;
                break;
            case ERROR_INVALID_NAME:
                cerr << "  -> Invalid port name format." << endl;
                break;
        }
#endif
        return false;
    }

    // Get current port configuration
    DCB dcbSerialParams = {0};  // Device Control Block
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (!GetCommState(hSerial, &dcbSerialParams)) {
#ifdef DEBUG
        cerr << "Error getting current port configuration" << endl;
#endif
        closePort();
        return false;
    }

    // Set serial port parameters (8N1 configuration)
    dcbSerialParams.BaudRate = baudRate;        // Baud rate (9600, 115200, etc.)
    dcbSerialParams.ByteSize = 8;               // 8 data bits
    dcbSerialParams.StopBits = ONESTOPBIT;      // 1 stop bit
    dcbSerialParams.Parity = NOPARITY;          // No parity bit
    dcbSerialParams.fDtrControl = DTR_CONTROL_ENABLE;  // Enable DTR
    dcbSerialParams.fRtsControl = RTS_CONTROL_ENABLE;  // Enable RTS

    // Apply the configuration
    if (!SetCommState(hSerial, &dcbSerialParams)) {
#ifdef DEBUG
        cerr << "Error setting port configuration" << endl;
#endif
        closePort();
        return false;
    }

    // Set timeout values (in milliseconds)
    COMMTIMEOUTS timeouts = {0};
    timeouts.ReadIntervalTimeout = 50;          // Max time between chars (ms)
    timeouts.ReadTotalTimeoutConstant = 50;     // Total read timeout base (ms)
    timeouts.ReadTotalTimeoutMultiplier = 10;   // Per-byte read timeout (ms)
    timeouts.WriteTotalTimeoutConstant = 50;    // Total write timeout (ms)
    timeouts.WriteTotalTimeoutMultiplier = 10;  // Per-byte write timeout (ms)
    
    if (!SetCommTimeouts(hSerial, &timeouts)) {
#ifdef DEBUG
        cerr << "Error setting timeout values" << endl;
#endif
        closePort();
        return false;
    }

    // Clear any existing data in the buffers
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

#ifdef DEBUG
    cout << "Successfully opened " << portName << " at " << baudRate << " baud (8N1)" << endl;
#endif
    return true;
}

// Send data over serial port
bool SerialPort::sendData(const string& data) {
    if (!isOpen()) {
#ifdef DEBUG
        cerr << "Port is not open" << endl;
#endif
        return false;
    }

    DWORD bytesWritten = 0;
    bool success = WriteFile(hSerial,                    // Handle to file
                            data.c_str(),               // Data to write
                            (DWORD)data.length(),       // Number of bytes to write
                            &bytesWritten,              // Number of bytes actually written
                            NULL);                      // Overlapped I/O (not used)

    if (!success) {
        DWORD error = GetLastError();
#ifdef DEBUG
        cerr << "Error writing to port (Error code: " << error << ")" << endl;
#endif
        return false;
    }

#ifdef DEBUG
    cout << "Sent " << bytesWritten << " bytes: \"" << data << "\"" << endl;
#endif
    return true;
}

// Read data from serial port
string SerialPort::readData(int maxBytes = 256) {
    if (!isOpen()) {
#ifdef DEBUG
        cerr << "Port is not open" << endl;
#endif
        return "";
    }

    char buffer[1024];  // Read buffer
    DWORD bytesRead = 0;
    string result;

    // Ensure we don't read more than buffer size
    if (maxBytes > sizeof(buffer) - 1) {
        maxBytes = sizeof(buffer) - 1;
    }

    bool success = ReadFile(hSerial,           // Handle to file
                           buffer,            // Buffer to store data
                           maxBytes,          // Maximum bytes to read
                           &bytesRead,        // Actual bytes read
                           NULL);             // Overlapped I/O (not used)

    if (success && bytesRead > 0) {
        buffer[bytesRead] = '\0';  // Null-terminate the string
        result = string(buffer, bytesRead);
#ifdef DEBUG
        cout << "Received " << bytesRead << " bytes: \"" << result << "\"" << endl;
#endif
    } else if (!success) {
        DWORD error = GetLastError();
#ifdef DEBUG
        cerr << "Error reading from port (Error code: " << error << ")" << endl;
#endif
    }
    // If bytesRead == 0, it just means no data was available (timeout)

    return result;
}

// Check if port is open and valid
bool SerialPort::isOpen() {
    return hSerial != INVALID_HANDLE_VALUE;
}

// Close the serial port
void SerialPort::closePort() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
#ifdef DEBUG
        cout << "Serial port closed" << endl;
#endif
    }
}
