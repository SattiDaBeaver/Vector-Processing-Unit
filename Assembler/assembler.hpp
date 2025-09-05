#pragma once

#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <unordered_map>

#define ERR_OOB 0x80000000;
#define ERR_INV 0x40000000;

using namespace std;

class Instruction {
public:
    string mnemonic;
    vector<string> operands;
    int line_number;
    int cycle;

    Instruction(string m, int c, int l);
};

class Assembler {
private:
    vector<Instruction> instructions;
    int currentLine = 0;
    int currentCycle = 0;

    // Bit Masks
    // Instruction OpCodes
    static constexpr uint32_t BIT_LOAD_LEFT     = (1u << 31);
    static constexpr uint32_t BIT_LOAD_TOP      = (1u << 30);
    static constexpr uint32_t BIT_SWAP_LEFT     = (1u << 29);
    static constexpr uint32_t BIT_SWAP_TOP      = (1u << 28);
    static constexpr uint32_t BIT_SHIFT_RIGHT   = (1u << 27);
    static constexpr uint32_t BIT_SHIFT_DOWN    = (1u << 26);
    static constexpr uint32_t BIT_LOAD_ACC      = (1u << 25);
    static constexpr uint32_t BIT_WRITE_ACC_OUT = (1u << 24);
    static constexpr uint32_t BIT_WAIT_CYCLES   = (1u << 23);
    static constexpr uint32_t BIT_JUMP          = (1u << 22);
    static constexpr uint32_t BIT_CLR           = (1u << 21);
    static constexpr uint32_t BIT_NOP           = (1u << 20);

    // Control and Clear Flags
    static constexpr uint32_t BIT_CLR_ACC       = (1u << 17);
    static constexpr uint32_t BIT_CLR_SYSTOLIC  = (1u << 16);
    static constexpr uint32_t BIT_CLR_LEFT_BUF  = (1u << 15);
    static constexpr uint32_t BIT_CLR_TOP_BUF   = (1u << 14);
    static constexpr uint32_t BIT_IMM_FLAG      = (1u << 13);

    // Operand / Immediate Field [12:0]

    // Hashmap of Operands
    const unordered_map<string, uint32_t> opcode_table = {
        {"LDL", BIT_LOAD_LEFT},
        {"LDLi", BIT_LOAD_LEFT},
        {"LDT", BIT_LOAD_TOP},
        {"LDTi", BIT_LOAD_TOP},
        {"LDLT", BIT_LOAD_LEFT | BIT_LOAD_TOP},
        {"SWL", BIT_SWAP_LEFT},
        {"SWT", BIT_SWAP_TOP},
        {"SHR", BIT_SHIFT_RIGHT},
        {"SHD", BIT_SHIFT_DOWN},
        {"LDA", BIT_LOAD_ACC},
        {"WAO", BIT_WRITE_ACC_OUT},
        {"WAIT", BIT_WAIT_CYCLES},
        {"JMP", BIT_JUMP},
        {"CLR", BIT_CLR},
        {"NOP", BIT_NOP},
        {"CLRA", BIT_CLR_ACC},
        {"CLRS", BIT_CLR_SYSTOLIC},
        {"CLRL", BIT_CLR_LEFT_BUF},
        {"CLRT", BIT_CLR_TOP_BUF},
    };

    // Methods
    bool ParseLine(const string& line);
    uint32_t AssembleInstruction(const Instruction& instruction);
    uint32_t GetValueFromString(const string& str_num);

public:
    Assembler();
    bool Assemble(const string& input_file, const string& output_file);
};