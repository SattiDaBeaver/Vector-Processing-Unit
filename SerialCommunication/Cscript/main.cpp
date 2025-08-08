#include "serialport.hpp"

#include <iostream>
#include <string>
#include <windows.h>

using namespace std;

// Example usage and testing
int main(int argc, char** argv) {
    SerialPort serial;
    
    // Common Windows COM port names: COM1, COM2, COM3, etc.
    // Check Device Manager -> Ports (COM & LPT) to see available ports
    string portName;
    if (argc < 2) {
        portName = "\\\\.\\COM18"; // Change this to match your device
    }
    else {
        portName = argv[1];  
        cout << argv[1] << endl;
    }
    int baudRate = 9600;            // Common baud rates: 9600, 19200, 38400, 57600, 115200

    cout << "Windows Serial Communication Test" << endl;
    cout << "=================================" << endl;

    // Try to open the serial port
    if (!serial.openPort(portName, baudRate)) {
        cerr << "Failed to open " << portName << endl;
        cout << "\nTips:" << endl;
        cout << "- Check Device Manager for correct COM port number" << endl;
        cout << "- Make sure no other program is using the port" << endl;
        cout << "- Verify the device is connected and drivers are installed" << endl;
        return -1;
    }

    // Test 1: Send some basic data
    cout << "\n--- Test 1: Sending basic data ---" << endl;
    serial.sendData("Hello World!\r\n");
    serial.sendData("AT\r\n");  // Common AT command for modems/modules

    // Wait for potential response
    Sleep(500);  // 500ms delay

    // Test 2: Try to read any incoming data
    cout << "\n--- Test 2: Reading incoming data ---" << endl;
    string response = serial.readData(100);
    cout << response << endl;
    if (response.empty()) {
        cout << "No data received (this is normal if nothing is connected)" << endl;
    }
     
    cout << "\n--- Test 3: Interactive mode ---" << endl;
    cout << "Type commands to send (or 'quit' to exit):" << endl;
    
    string userInput;
    while (true) {
        cout << "Send> ";
        getline(cin, userInput);
        
        if (userInput == "quit" || userInput == "exit") {
            break;
        }
        
        // Send user input (add line ending if not present)
        if (!userInput.empty()) {
            if (userInput.back() != '\n') {
                userInput += "\r\n";
            }
            serial.sendData(userInput);
            
            // Wait a bit and try to read response
            Sleep(200);
            serial.readData(256);
        }
    }
     
    cout << "\nTest completed. Port will be closed automatically." << endl;
    
    // Port is automatically closed by destructor
    return 0;
}