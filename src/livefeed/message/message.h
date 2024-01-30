
#include <iostream>
#include <stdint.h>
#include "hex.h"
#include <vector>
#include <string>
#include <map>
#include "command.h"

struct Data {
    string data;
    string id;
};

class Message {
    public:

        Data firmareVersion();               
        Data hardwareId();               
        Data gimbalInfo();               
        Data funcFeedback();               
        
        Data takePhoto();               
        Data record();               

        Data autoFocus();               
        Data centerGimbal();               
        Data lock();               
        Data follow();               
        Data fpv();               
        Data gimbalAttitude();               
        
        Data zoomIn();               
        Data zoomOut();               
        Data stopZoom();               

        Data longFocus();               
        Data closeFocus();               
        Data stopFocus();               

        Data gimbalSpeed();               


};