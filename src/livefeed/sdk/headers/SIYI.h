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
#include <thread>
#include <mutex> 

#define BUFFER_SIZE 1024 
#define MAX_TIMEOUT 5
#define CHECK_CONNECTION_INTERVAL 100 // ms
#define RETRY_CONNECTION_INTERVAL 1000 // ms

using namespace std;

struct SequenceState {
    uint16_t seq = 0;
    uint16_t lastSeq = 0;
};

struct ZoomState : SequenceState {
    int level = -1;
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
    ZoomState zoom;
    AttitudeState attitude;
};

class SIYI {

    private:
        State state;

        int socketfd;
        string ip;
        int port;
        struct sockaddr_in servaddr; 

        // Queue for sending messages
        SafeQueue<string> queue;

        vector<thread> threads;

        atomic<bool> connected = false; 
        atomic<bool> active = false;

        void checkConnection();

        void connectionLoop();
        void recvLoop();

        void gimbalInfoLoop();
        void gimbalAttitudeLoop();

    public:

        SIYI(string ip = "192.168.144.25", int port = 37260);

        bool connect(int maxRetries = 3);
        void disconnect();

        void requestGimbalAttitude();
        void requestGimbalInfo();

        void requestZoomIn();
        void requestZoomOut();
        void requestZoomHold();

        void requestLongFocus();
        void requestCloseFocus();
        void requestHoldFocus();

        void requestCenterGimbal();
        void requestGimbalSpeed();

        void bufferCallback();
        void emit(string message);
        void emitAll();

        void setGimbalRotation(double yaw, double pitch, double errorThreshold, double Kp = 4);

};