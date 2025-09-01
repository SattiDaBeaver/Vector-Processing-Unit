#include "assembler.hpp"
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>

using namespace std;

int main(int argc, char* argv[]) {
    // Check for correct arguments
    if (argc != 3) {
        cerr << "Usage: " << argv[0] << " <input_asm_file> <output_bin_file>\n";
        return 1;
    }

    // Get input/output file names
    string input_file = argv[1];
    string output_file = argv[2];

    Assembler assembler;

    if (!assembler.Assemble(input_file, output_file)) {
        cerr << "Assembly failed.\n";
        return 1;
    }

    std::cout << "Assembly succeeded!\n";
    return 0;
}