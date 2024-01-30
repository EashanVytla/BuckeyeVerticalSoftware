#include "hex.h"
#include <iostream>
#include <stdint.h>
#include <string>


using namespace std;

int main() {

    string hex = "556601000000000a";

    vector<char> v = fromHex(hex);

    for (size_t i = 0; i < v.size(); i++)
        cout << static_cast<int>(static_cast<unsigned char>(v.at(i))) << ", ";
        
    cout << string("a") + "b" << endl;


    cout << toHex(1) << endl;

    cout << hex.substr(0, 2) << hex.substr(2, 2) << endl;


    return 0;

    // std::string s = std::format("{:x}", 42);
}