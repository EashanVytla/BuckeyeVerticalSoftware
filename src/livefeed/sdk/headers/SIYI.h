// #include <bits/stdc++.h> 

#pragma once

#include <stdlib.h> 
#include <unistd.h> 
#include <string.h> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <netinet/in.h> 
#include <string>
#include <safequeue.h>
#include <messages.h>
#include <thread>
#include <mutex> 

#define BUFFER_SIZE 1024 
#define MAX_TIMEOUT 5

#define MAX_RETRIES 5

#define CHECK_CONNECTION_INTERVAL 1000 // ms

#define RETRY_CONNECTION_INTERVAL 1000 // ms
// #define RETRY_CONNECTION_INTERVAL 1000 // ms

#define SET_GIMBAL_INTERVAL 100 // ms
#define GIMBAL_INFO_INTERVAL 1000 // ms
#define GIMBAL_ATTITUDE_INTERVAL 100 // ms

#define MIN_RECV_BYTES 10

using namespace std;

struct SequenceState {
    uint16_t seq = 0;
    uint16_t lastSeq = 0;
};

struct FirmwareState : SequenceState {
    string version = "";
};

struct ZoomState : SequenceState {
    double level = -1.0;
};

struct MotionState : SequenceState {
    int mode = -1;
    bool isLock() { return mode == 0; }
    bool isFollow() { return mode == 1; }
    bool isFPV() { return mode == 1; }
};

struct AttitudeState : SequenceState {
    int stamp = 0;

    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;

    double yawSpeed = 0.0;
    double pitchSpeed = 0.0;
    double rollSpeed = 0.0;

};

struct State {
    FirmwareState firmware;
    ZoomState zoom;
    AttitudeState attitude;
    MotionState motion;
};

class SIYI {

    private:
        State state;

        int socketfd;
        string ip;
        int port;
        struct sockaddr_in servaddr; 

        // Queue for sending messages
        SafeQueue<string> socketQueue;
        vector<thread> threads;
        vector<char> buffer = vector<char>(BUFFER_SIZE);

        atomic<bool> connected = false; 
        atomic<bool> active = false;

        mutex stateMut;  

        void connectionLoop();
        void recvLoop();
        void gimbalInfoLoop();
        void gimbalAttitudeLoop();

        void emit(string message);
        void emitAll();
        void requestEmit(Data encodedMessage);

        void bufferCallback();
        void checkConnectionCallback();

        // parse functions

        void parseFirmwareVersion(string message, uint16_t seq);

        void parseGimbalAttitude(string message, uint16_t seq);
        void parseGimbalInfo(string message, uint16_t seq);

        // void parseGimbalSpeed(string message, uint16_t seq);
        // void parseGimbalCenter(string message, uint16_t seq);
        // void parseManualFocus(string message, uint16_t seq);

        void parseZoom(string message, uint16_t seq);


    public:

        SIYI(string ip = "192.168.144.25", int port = 37260);

        bool connect();
        void disconnect();

        // request functions

        void requestFirmwareVersion();

        void requestGimbalAttitude();
        void requestGimbalInfo();

        void requestZoomIn();
        void requestZoomOut();
        void requestZoomHold();

        void requestLongFocus();
        void requestCloseFocus();
        void requestHoldFocus();

        void requestMotionLock();
        void requestMotionFPV();
        void requestMotionFollow();

        void requestCenterGimbal();
        void requestGimbalSpeed(int yawSpeed, int pitchSpeed);

        // sets
        // TODO: setZoomLevel()
        void setGimbalRotation(double yaw, double pitch, double errorThreshold = 1.0, double Kp = 4);

        // gets

        FirmwareState getFirmwareState();
        AttitudeState getAttitudeState();
        MotionState getMotionState();
        ZoomState getZoomState();

};