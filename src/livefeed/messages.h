
#include <iostream>
#include <stdint.h>
#include "hex.h"
#include <vector>
#include <string>
#include <map>



class Messages {
    public:
        const string GET_FIRMWARE_VERSION = "01";
        const string GET_HARDWARE_ID = "02";

        const string AUTO_FOCUS = "04";
        const string MANUAL_ZOOM = "05";
        const string MANUAL_FOCUS = "06";
        const string GIMBAL_ROT = "07";
        const string CENTER = "08";

        const string GET_GIMBAL_INFO = "0A";
        const string FUNC_FEEDBACK_INFO = "0B";
        const string PHOTO_VIDEO_HDR = "0C";
        const string GET_GIMBAL_ATT = "0D";

        const map<string, string> data = {
            {Messages::GET_FIRMWARE_VERSION, ""},
            {Messages::GET_HARDWARE_ID, ""},

            {Messages::AUTO_FOCUS, ""},
            {Messages::MANUAL_ZOOM, ""},
            {Messages::MANUAL_FOCUS, ""},
            {Messages::GIMBAL_ROT, ""},
            {Messages::CENTER, ""},

            {Messages::GET_GIMBAL_INFO, ""},
            {Messages::FUNC_FEEDBACK_INFO, ""},
            {Messages::PHOTO_VIDEO_HDR, ""},
            {Messages::GET_GIMBAL_ATT, ""},
        };

};


