#include "Bytes.h"
#include <iostream>
#include <string>


using namespace std;

int main() {

    Bytes bytes;

    string hex = "556601000000000a";

    vector<char> v = bytes.fromHex(hex);

    for (int i = 0; i < v.size(); i++)
        cout << static_cast<int>(static_cast<unsigned char>(v.at(i))) << " ";
        
    cout << endl;

    return 0;
}