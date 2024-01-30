#include <iostream>
#include <sstream> 
#include <stdint.h>
#include <iomanip>
#include "hex.h"

using namespace std;

string toHex(int val, int nbytes) {

    int bitmask = (1 << (nbytes * 8)) - 1;
    int result = (val + (1 << (nbytes * 8))) & bitmask;

    std::stringstream stream;
    stream << std::hex << setw((nbytes * 8) / 4) << setfill('0') << result;

    string str = stream.str();

    // Ensures string is of even numbered length
    if (str.length() == 1)
        str = string("0") + str;

    return str;
}


vector<char> fromHex(string& str) {
    vector<char> bytes;

    for (size_t i = 0; i < str.length(); i += 2) {

        char byte = static_cast<char>(
            stoi(str.substr(i, 2), 0, 16)
        );

        bytes.push_back(byte);
    }


  return bytes;
}