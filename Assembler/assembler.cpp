#include "assembler.hpp"

// Instruction Class

Instruction::Instruction(string m, int c) {
    mnemonic = m;
    cycle = c;
}


// Assembler Class

Assembler::Assembler () {

}

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

    Instruction instr(mnemonic, currentCycle);
    instr.operands = operands;
    instructions.push_back(instr);

    return true;
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

    // Optional: print parsed instructions for testing
    for (auto &instr : instructions) {
        cout << "Cycle " << instr.cycle << ": " << instr.mnemonic;
        for (auto &op : instr.operands) cout << " " << op;
        cout << "\n";
    }

    return true;
}

