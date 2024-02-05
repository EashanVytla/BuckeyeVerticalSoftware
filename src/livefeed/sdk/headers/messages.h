#pragma once


#include <iostream>
#include <stdint.h>
#include "hex.h"
#include <vector>
#include <string>
#include <map>

#define HEADER "5566"
#define TWO_BYTES_ZERO "0000"
#define ONE  "01"

struct Data {
    string data;
    string commandId = "";
    uint16_t seq = 0;
    bool success = true; // if data was extracted successfully
};

class Message {
    public:

        static Data firmareVersion();               
        static Data hardwareId();               
        static Data gimbalInfo();               
        static Data funcFeedback();               
        
        static Data takePhoto();               
        static Data record();               

        static Data autoFocus();               
        static Data centerGimbal();               
        static Data lock();               
        static Data follow();               
        static Data fpv();               
        static Data gimbalAttitude();               
        
        static Data zoomIn();               
        static Data zoomOut();               
        static Data stopZoom();               

        static Data longFocus();               
        static Data closeFocus();               
        static Data stopFocus();               

        static Data gimbalSpeed(int yawSpeed, int pitchSpeed);               

        static Data encode(Data message);

        static Data decode(string encoded);

};