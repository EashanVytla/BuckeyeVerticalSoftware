#include <iostream>
#include <stdint.h>
#include "hex.h"
#include "crc.h"
#include <vector>

uint16_t crc16(vector<char> bytes) {

    uint16_t result = 0x0;

    for (size_t i = 0; i < bytes.size(); i++) {

        uint8_t byte = static_cast<uint8_t>(bytes.at(i));

        result = ((result<<8)& 0xff00) ^ table[((result>>8) & 0xff) ^ byte];

    }

    result = result & 0xffff;

    return result;

}