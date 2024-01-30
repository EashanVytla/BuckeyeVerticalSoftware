#pragma once


#include <iostream>
#include <stdint.h>
#include <vector>

using namespace std;

string toHex(int val, int nbytes);
vector<char> fromHex(string& hex); 