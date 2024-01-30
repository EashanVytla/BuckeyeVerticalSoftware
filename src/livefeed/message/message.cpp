
#include <iostream>
#include <stdint.h>
#include "hex.h"
#include <vector>
#include <string>
#include "message.h"
#include "crc.h"
#include "commandType.h"


string Message::encode(Data message) {

    const string HEADER = "5566";
    const string TWO_BYTES_ZERO = "0000";
    const string ONE  = "01";

    string hex = "";

    // Number of bytes
    string dataLength = toHex(message.data.length() / 2, 2);

    // reorder bits
    dataLength = dataLength.substr(2, 2) + dataLength.substr(0, 2);

    hex += HEADER; 
    hex += ONE; 
    hex += dataLength; 
    hex += TWO_BYTES_ZERO; 
    hex += message.commandId; 
    hex += message.data; 

    uint16_t check = crc16(fromHex(hex));

    string crcString = toHex(check, 2);

    // reverse low and high byte
    crcString = crcString.substr(2, 2) + crcString.substr(0, 2);

    hex += crcString;

    return hex;

}

// Message functions

Data Message::firmareVersion() {
    return { "", Command::GET_FIRMWARE_VERSION};
}

Data Message::hardwareId() {
    return { "", Command::GET_HARDWARE_ID};
}

Data Message::gimbalInfo() {
    return { "", Command::GET_GIMBAL_INFO};
}

Data funcFeedback() {
    return { "", Command::FUNC_FEEDBACK_INFO};
}

Data Message::takePhoto() {
    return { "00", Command::PHOTO_VIDEO_HDR};
}

Data Message::record() {
    return { "02", Command::PHOTO_VIDEO_HDR};
}

Data Message::autoFocus() {
    return { "01", Command::AUTO_FOCUS};
}

Data Message::centerGimbal() {
    return { "01", Command::CENTER};
}

Data Message::lock() {
    return { "03", Command::PHOTO_VIDEO_HDR};
}

Data Message::follow() {
    return { "04", Command::PHOTO_VIDEO_HDR};
}

Data Message::fpv() {
    return { "05", Command::PHOTO_VIDEO_HDR};
}

Data Message::gimbalAttitude() {
    return { "", Command::GET_GIMBAL_ATT};
}

Data Message::zoomIn() {
    return { "01", Command::MANUAL_ZOOM};
}

Data Message::zoomOut() {
    return { "ff", Command::MANUAL_ZOOM};
}

Data Message::stopZoom() {
    return { "00", Command::MANUAL_ZOOM};
}

Data Message::longFocus() {
    return { "01", Command::MANUAL_FOCUS};
}

Data Message::closeFocus() {
    return { "ff", Command::MANUAL_FOCUS};
}

Data Message::stopFocus() {
    return { "00", Command::MANUAL_FOCUS};
}

Data Message::gimbalSpeed(int yawSpeed, int pitchSpeed) {

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
        toHex(yawSpeed, 1) + toHex(pitchSpeed, 1),
        Command::GIMBAL_ROT
    };

}
