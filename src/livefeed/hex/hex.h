#pragma once


#include <iostream>
#include <stdint.h>
#include <vector>

using namespace std;

class Hex {
    public:
        static string toHex(int val, int nbytes);
        static vector<char> asVector(string hex); 
        static int asInt(string hex); 
};

