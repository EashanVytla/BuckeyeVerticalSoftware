#pragma once


#include <iostream>
#include <stdint.h>
#include "hex.h"
#include <vector>
#include <string>
#include <map>

struct Data {
    string data;
    string commandId;
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

        static string encode(Data message);

};