#include <iostream>
#include <stdint.h>

using namespace std;

class Bytes {
    private: 
    public:
        std::vector<char> fromHex(const std::string& hex); 
};


std::vector<char> Bytes::fromHex(const std::string& hex) {
    std::vector<char> bytes;

    for (size_t i = 0; i < hex.length(); i += 2) {

        char byte = static_cast<char>(
            stoi(hex.substr(i, 2), 0, 16)
        );

        bytes.push_back(byte);
    }


  return bytes;
}