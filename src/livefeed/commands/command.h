
#include <iostream>
#include <string>

using namespace std;

class Command {
    public:
        inline static const string GET_FIRMWARE_VERSION = "01";
        inline static const string GET_HARDWARE_ID = "02";

        inline static const string AUTO_FOCUS = "04";
        inline static const string MANUAL_ZOOM = "05";
        inline static const string MANUAL_FOCUS = "06";
        inline static const string GIMBAL_ROT = "07";
        inline static const string CENTER = "08";

        inline static const string GET_GIMBAL_INFO = "0A";
        inline static const string FUNC_FEEDBACK_INFO = "0B";
        inline static const string PHOTO_VIDEO_HDR = "0C";
        inline static const string GET_GIMBAL_ATT = "0D";
};