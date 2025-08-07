#include "serialport.hpp"

SerialPort::SerialPort()  {
    hSerial = INVALID_HANDLE_VALUE;
}

bool SerialPort::openPort(const string& portName, int baudRate = 9600) {
    // Open the serial port
    // Use CreateFileA for ANSI string, CreateFileW for wide strings
    hSerial = CreateFileA(portName.c_str(),
                            GENERIC_READ | GENERIC_WRITE,  // Read and write access
                            0,                             // No sharing
                            NULL,                          // Default security attributes
                            OPEN_EXISTING,                 // Port must exist
                            FILE_ATTRIBUTE_NORMAL,         // Normal file attributes
                            NULL);                         // No template file

    if (hSerial == INVALID_HANDLE_VALUE) {
        DWORD error = GetLastError();
        cerr << "Error opening port " << portName << " (Error code: " << error << ")" << endl;
        return false;
    }

    // Get current port configuration
    DCB dcbSerialParams = {0};  // Device Control Block
    dcbSerialParams.DCBlength = sizeof(dcbSerialParams);
    
    if (!GetCommState(hSerial, &dcbSerialParams)) {
        cerr << "Error getting current port configuration" << endl;
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
        cerr << "Error setting port configuration" << endl;
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
        cerr << "Error setting timeout values" << endl;
        closePort();
        return false;
    }

    // Clear any existing data in the buffers
    PurgeComm(hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);

    cout << "Successfully opened " << portName << " at " << baudRate << " baud (8N1)" << endl;
    return true;
}

bool SerialPort::sendData(const string& data) {
    if (!isOpen()) {
        cerr << "Port is not open" << endl;
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
        cerr << "Error writing to port (Error code: " << error << ")" << endl;
        return false;
    }

    cout << "Sent " << bytesWritten << " bytes: \"" << data << "\"" << endl;
    return true;
}

string SerialPort::readData(int maxBytes = 256) {
    if (!isOpen()) {
        cerr << "Port is not open" << endl;
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
        cout << "Received " << bytesRead << " bytes: \"" << result << "\"" << endl;
    } else if (!success) {
        DWORD error = GetLastError();
        cerr << "Error reading from port (Error code: " << error << ")" << endl;
    }
    // If bytesRead == 0, it just means no data was available (timeout)

    return result;
}

bool SerialPort::isOpen() {
    return hSerial != INVALID_HANDLE_VALUE;
}

void SerialPort::closePort() {
    if (hSerial != INVALID_HANDLE_VALUE) {
        CloseHandle(hSerial);
        hSerial = INVALID_HANDLE_VALUE;
        cout << "Serial port closed" << endl;
    }
}

SerialPort::~SerialPort() {
    closePort();
}
