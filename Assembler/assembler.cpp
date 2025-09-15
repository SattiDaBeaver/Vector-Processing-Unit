#include "assembler.hpp"

// Instruction Class
Instruction::Instruction(string m, int c, int l) {
    mnemonic = m;
    cycle = c;
    line_number = l;
}


// Assembler Class
Assembler::Assembler () {

}

// Get Value from String
uint32_t Assembler::GetValueFromString(const string& str_num) {
    string str = str_num;
    int32_t num;
    if (str.rfind("0x", 0) == 0 || str.rfind("0X", 0) == 0) {
        // Hex
        num = static_cast<int32_t>(stoul(str, nullptr, 16));
    } else {
        // Decimal
        num = static_cast<int32_t>(stoul(str, nullptr, 10));
    }

    return (uint32_t) num;
}

// Parse Line 
bool Assembler::ParseLine (const string& line) {
    string l = line;

    // Remove Comments
    size_t comment_position = l.find(';');
    // if found comment
    if (comment_position != string::npos) {
        l = l.substr(0, comment_position);
    }

    // Remove leading/trailing whitespaces
    l.erase(0, l.find_first_not_of(" \t\r\n")); // remove leading whitespace
    l.erase(l.find_last_not_of(" \t\r\n") + 1); // remove trailing whitespace (till end of line)

    // Check for empty line
    if (l.empty()) {
        return true;
    }

    // Check for next cycle symbol ($)
    if (l[0] == '$') {
        if (l.length() > 1) {
            cerr << "Warning: text after '$' is ignored in line number:" << currentLine << "\n" << l << "\n";
        }
        currentCycle++;
        return true;
    }

    // Get mnemonic
    istringstream line_stream(l);
    string token;
    line_stream >> token;
    string mnemonic = token; 

    vector<string> operands;

    while (line_stream >> token) {
        // Remove trailing comma if present
        if (!token.empty() && token.back() == ',') {
            token.pop_back();
        }
        operands.push_back(token);
    }

    Instruction instr(mnemonic, currentCycle, currentLine);
    instr.operands = operands;
    instructions.push_back(instr);

    return true;
}

// Assemble Instructions
uint32_t Assembler::AssembleInstruction (const Instruction& instruction){
    // Check for valid mnemonic
    auto iterator = opcode_table.find(instruction.mnemonic);
    if (iterator == opcode_table.end()) {
        cout << "Error: Unknown mnemonic \"" << instruction.mnemonic << 
        "\" on line " << instruction.line_number << "\n";
        return 0u;  // return no instruction (invalid)
    }

    // Set the opcode bits - all bits except the immediates and stuff
    uint32_t instr_word = 0u;
    instr_word |= iterator->second;

    bool is_imm = false;
    

    // WAO instruction
    if (instruction.mnemonic == "WAO") {

    }
    // LDL, LDT and WAIT instructions
    if (instruction.mnemonic == "LDL" || instruction.mnemonic == "LDT" ||
        instruction.mnemonic == "LDLi" || instruction.mnemonic == "LDTi" ||
        instruction.mnemonic == "LDLT"
    ) {
        // Check for immediate
        if (instruction.mnemonic.back() == 'i') {
            instr_word |= BIT_IMM_FLAG;
            is_imm = true;
        }

        if (instruction.operands.size() < 2) {
            cout << "Error: Too few operands (found " << instruction.operands.size() << 
            ", expected 2) on line " << instruction.line_number << "\n";
            
            return 0u; // return error
        } 
        else if (instruction.operands.size() > 2) {
            cout << "Error: Too many operands (found " << instruction.operands.size() << 
            ", expected 2) on line " << instruction.line_number << "\n";
            
            return 0u; // return error
        }

        uint32_t sys_addr = GetValueFromString(instruction.operands[0]);
        uint32_t mem_addr = GetValueFromString(instruction.operands[1]);

        // Out of Bounds Check
        // Check systolic address
        if (sys_addr > 8) {
            cout << "Error: Systolic array address out of bounds (" << sys_addr << 
            " > 8) on line " << instruction.line_number << "\n";
            return 0u; // return error
        }
        // Check Immediate Value
        if (is_imm) {
            if (((int32_t) mem_addr) > 127 || ((int32_t) mem_addr) < -128 ) { // Immediate Value
                cout << "Error: Immediate value out of bounds (" << ((int32_t) mem_addr) << 
                " > 127 or < -128) on line " << instruction.line_number << "\n";
                return 0u;
            } 
        }
        // Check Address
        else {
            if (mem_addr > 4095) { // Address
                cout << "Error: Address out of bounds (" << mem_addr << 
                " > 4095 on line " << instruction.line_number << "\n";
                return 0u;
            } 
        }

        // Change here, add error checking, OOB checks

        // Create Instruction Word
        instr_word |= ((sys_addr & 0xF) << 10) | (mem_addr & 0xFFF);
    }
    
    return instr_word;
}

bool Assembler::Assemble(const string& input_file, const string& output_file) {
    ifstream in_stream(input_file);
    // Check if input file exists
    if (!in_stream.is_open()) {
        cerr << "Error: Cannot open input file " << input_file << "\n";
        return false;
    }

    ofstream out_stream(output_file, ios::binary);
    // Check if output file exists
    if (!out_stream.is_open()) {
        cerr << "Error: Cannot open input file " << input_file << "\n";
        return false;
    }

    // Parse File
    currentLine = 0;
    currentCycle = 0;
    string line;

    while (getline(in_stream, line)) {
        currentLine++;
        if (!ParseLine(line)) {
            cerr << "Error parsing line " << currentLine << ": " << line << "\n";
            return false;
        }
    }

    // Debugging
    for (auto &instr : instructions) {
        cout << instr.line_number << ". Cycle " << instr.cycle << ": " << instr.mnemonic;
        for (auto &op : instr.operands) {
            cout << " " << op;
        }
        cout << "\n";
    }

    // Encode and Assemble Instructions
    uint32_t instr_word = 0;
    int prev_cycle = 0;
    for (auto &instr : instructions) {
        uint32_t curr_word = AssembleInstruction(instr);
        if (curr_word == 0) {
            return false;
        }
        
        if (prev_cycle != instr.cycle) {
            instr_word = curr_word;
            prev_cycle = instr.cycle;
        }
        else {
            instr_word |= curr_word;
        }
        // Debug print for now
        cout << "Line " << instr.line_number
            << " (Cycle " << instr.cycle << "): "
            << instr.mnemonic << " -> 0x"
            << hex << instr_word << dec << "\n";
    }

    

    return true;
}

