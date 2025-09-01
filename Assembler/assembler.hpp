#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

class Instruction {
public:
    string mnemonic;
    vector<string> operands;
    int cycle;

    Instruction(string m, int c);
};

class Assembler {
private:
    vector<Instruction> instructions;
    int currentLine = 0;
    int currentCycle = 0;

    bool ParseLine(const string& line);

public:
    Assembler();
    bool Assemble(const string& input_file, const string& output_file);
};