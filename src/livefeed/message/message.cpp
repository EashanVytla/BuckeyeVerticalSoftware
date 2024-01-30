


#include <iostream>
#include <stdint.h>
#include "hex.h"
#include <vector>
#include <string>
#include <map>
#include "command.h"
#include "message.h"





// Message functions

Data firmareVersion() {
    return { "", Command::GET_FIRMWARE_VERSION};
}

Data hardwareId() {
    return { "", Command::GET_HARDWARE_ID};
}

Data gimbalInfo() {
    return { "", Command::GET_GIMBAL_INFO};
}

Data funcFeedback() {
    return { "", Command::FUNC_FEEDBACK_INFO};
}

Data takePhoto() {
    return { "00", Command::PHOTO_VIDEO_HDR};
}

Data record() {
    return { "02", Command::PHOTO_VIDEO_HDR};
}

Data autoFocus() {
    return { "01", Command::AUTO_FOCUS};
}

Data centerGimbal() {
    return { "01", Command::CENTER};
}

Data lock() {
    return { "03", Command::PHOTO_VIDEO_HDR};
}

Data follow() {
    return { "04", Command::PHOTO_VIDEO_HDR};
}

Data fpv() {
    return { "05", Command::PHOTO_VIDEO_HDR};
}

Data gimbalAttitude() {
    return { "", Command::GET_GIMBAL_ATT};
}

Data zoomIn() {
    return { "01", Command::MANUAL_ZOOM};
}

Data zoomOut() {
    return { "ff", Command::MANUAL_ZOOM};
}

Data stopZoom() {
    return { "00", Command::MANUAL_ZOOM};
}

Data longFocus() {
    return { "01", Command::MANUAL_FOCUS};
}

Data closeFocus() {
    return { "ff", Command::MANUAL_FOCUS};
}

Data stopFocus() {
    return { "00", Command::MANUAL_FOCUS};
}

Data gimbalSpeed(int yawSpeed, int pitchSpeed) {

    // Ensure speed within bounds
    if (yawSpeed > 100)
        yawSpeed = 100;

    if (yawSpeed < -100)
        yawSpeed = -100;

    // Ensure speed within bounds
    if (pitchSpeed > 100)
        pitchSpeed = 100;

    if (pitchSpeed < -100)
        pitchSpeed = -100;

    return {
        toHex(yawSpeed) + toHex(pitchSpeed),
        Command::GIMBAL_ROT
    };

}
