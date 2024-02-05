// #include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <netinet/in.h> 
#include <mutex> 
#include <thread>
#include <chrono>

#include <SIYI.h>
#include <hex.h>
#include <messages.h>


using namespace std;


SIYI::SIYI(string ip, int port) {

    SIYI::port = port;
    SIYI::ip = ip;

};

bool SIYI::connect() {

    active = true;
    connected = false;

    socketfd = socket(AF_INET, SOCK_DGRAM, 0);

    if (socketfd < 0) {
        perror("ERROR: couldn't create socket");
        active = false;
        return false;
    }

    // fills servaddr with 0s
    memset(&servaddr, 0, sizeof(servaddr)); 

    // Filling server information 
    servaddr.sin_family = AF_INET; 
    servaddr.sin_port = htons(port); 
    servaddr.sin_addr.s_addr = inet_addr(ip.c_str()); 

     // Set timeout for recvfrom using setsockopt
    timeval timeout;
    timeout.tv_sec = MAX_TIMEOUT;
    timeout.tv_usec = 0;

    if (setsockopt(socketfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1) {
        perror("setsockopt failed");

        SIYI::disconnect(); 
        return false;
    }

    thread t_recv(&SIYI::recvLoop, this);
    thread t_connection(&SIYI::connectionLoop, this);

    threads.push_back(std::move(t_recv));
    threads.push_back(std::move(t_connection));


    int counter = 0;

    while (counter < MAX_RETRIES) {
        
        if (connected) {

            thread t_recv(&SIYI::recvLoop, this);
            thread t_connection(&SIYI::connectionLoop, this);

            threads.push_back(std::move(t_recv));
            threads.push_back(std::move(t_connection));

            return true;

        }

        // wait a second before retrying
        this_thread::sleep_for(chrono::milliseconds(RETRY_CONNECTION_INTERVAL));

        counter++;

    }

    SIYI::disconnect();

    return false;

}

// CAUTION: only allow disconnect() to be called in parent thread
void SIYI::disconnect() {

    cout << "now disconnecting..." << endl;

    active = false;
    connected = false;

    // unique_lock<mutex> lock(stateMut);

    for (auto&& t : threads) {
        if (t.joinable()) {
            t.join();
        }
    }

    threads.clear();

    close(socketfd);

    // prevent reuse
    socketfd = -1;


    // TEMP, remove later
    // this_thread::sleep_for(chrono::seconds(3));

 
}

// socket loops

void SIYI::connectionLoop() {

    // connected = false;

    int counter = -1;

    while (active && counter < MAX_RETRIES) {

        SIYI::requestFirmwareVersion();

        {
            unique_lock<mutex> lock(stateMut); 

            FirmwareState &firmware = state.firmware;

            if (firmware.seq != firmware.lastSeq) {
                firmware.lastSeq = firmware.seq;
                counter = -1;

                connected = true;
            } 

            counter++;

        }

        this_thread::sleep_for(chrono::milliseconds(CHECK_CONNECTION_INTERVAL));
    }

    connected = false;

}

void SIYI::recvLoop() {

    while (active) {
        try {

            cout << "inside recvLoop" << endl;
        
            SIYI::bufferCallback();

            this_thread::sleep_for(chrono::milliseconds(1000));

        } catch (exception &e) {

            cout << e.what() << endl;
        }

    }

}

void SIYI::gimbalInfoLoop() {
    while (active) {
        SIYI::requestGimbalInfo();
        this_thread::sleep_for(chrono::microseconds(GIMBAL_INFO_INTERVAL));
    }
}

void SIYI::gimbalAttitudeLoop() {
    while (active) {
        SIYI::requestGimbalAttitude();
        this_thread::sleep_for(chrono::microseconds(GIMBAL_ATTITUDE_INTERVAL));
    }
}


// TODO
void SIYI::bufferCallback() {
    cout << "bufferCallback" << endl;

    socklen_t addrlen = sizeof(servaddr);
    ssize_t nbytes = recvfrom(socketfd, buffer.data(), BUFFER_SIZE, 0, (sockaddr*)&servaddr, &addrlen);

    if (nbytes < 0) {
        cout << "error reading bytes" << endl;
        return;
    }

    if (nbytes < MIN_RECV_BYTES) {
        cout << "error: too little bytes" << endl;
        return;
    }

    string encodedMessage = Hex::toHex(buffer, nbytes);


    while (static_cast<int>(encodedMessage.length()) >= MIN_RECV_BYTES * 2) {

        if (encodedMessage.substr(0, 4) != string(HEADER)) {
            encodedMessage = encodedMessage.substr(1, encodedMessage.length() - 1);
            continue;
        }


        int charLength = 2 * Hex::asInt(encodedMessage.substr(8, 2) + encodedMessage.substr(6, 2));

        if (static_cast<int>(encodedMessage.length()) < MIN_RECV_BYTES * 2 + charLength) {
            cout << "nothing useful" << endl;
            break;
        }

        Data decoded = Message::decode(encodedMessage);

        if (!decoded.success) {
            continue;
        }

        

        // if 
    }

}

// TODO
void SIYI::checkConnectionCallback() {
    cout << "checkConnection" << endl;
}

void SIYI::emit(string message) {

    try {
        vector<char> v = Hex::asVector(message);

        if (sendto(socketfd,  (const void*)v.data(), v.size(), 0, (sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
            cout << "error emitting message" << endl;
        }

    } catch (exception &e) {
        cout << "Exception: " << e.what() << endl;
    }

}

void SIYI::emitAll() {
    while (socketQueue.size() > 0) {
        emit(socketQueue.dequeue());
    }
}

void SIYI::requestEmit(Data encodedMessage) {

    if (!encodedMessage.success) {
        return;
    }

    socketQueue.enqueue(encodedMessage.data);
    emitAll();

}



// Request functions

void SIYI::requestFirmwareVersion() {
    requestEmit(Message::encode(
        Message::firmareVersion()
    ));
}

void SIYI::requestGimbalAttitude() {
    requestEmit(Message::encode(
        Message::gimbalAttitude()
    ));
}


void SIYI::requestGimbalInfo() {
    requestEmit(Message::encode(
        Message::gimbalInfo()
    ));
}


void SIYI::requestZoomIn() {
    requestEmit(Message::encode(
        Message::zoomIn()
    ));
}

void SIYI::requestZoomOut() {
    requestEmit(Message::encode(
        Message::zoomOut()
    ));
}

void SIYI::requestZoomHold() {
    requestEmit(Message::encode(
        Message::stopZoom()
    ));
}

void SIYI::requestLongFocus() {
    requestEmit(Message::encode(
        Message::longFocus()
    ));
}

void SIYI::requestCloseFocus() {
    requestEmit(Message::encode(
        Message::closeFocus()
    ));
}

void SIYI::requestHoldFocus() {
    requestEmit(Message::encode(
        Message::stopFocus()
    ));
} 


void SIYI::requestMotionFPV() {
    requestEmit(Message::encode(
        Message::fpv()
    ));
}

void SIYI::requestMotionLock() {
    requestEmit(Message::encode(
        Message::lock()
    ));
}

void SIYI::requestMotionFollow() {
    requestEmit(Message::encode(
        Message::follow()
    ));
} 

void SIYI::requestCenterGimbal() {
    requestEmit(Message::encode(
        Message::centerGimbal()
    )); 
}

void SIYI::requestGimbalSpeed(int yawSpeed, int pitchSpeed) {
    requestEmit(Message::encode(
        Message::gimbalSpeed(yawSpeed, pitchSpeed)
    )); 
}


// parse functions

void SIYI::parseFirmwareVersion(string message, uint16_t seq) {

    unique_lock<mutex> lock(stateMut);  

    FirmwareState &firmware = state.firmware;

    firmware.version = message.substr(8, 8);
    firmware.seq = seq;
}

void SIYI::parseGimbalAttitude(string message, uint16_t seq) {

    unique_lock<mutex> lock(stateMut);  

    AttitudeState &attitude = state.attitude;

    attitude.seq = seq;

    attitude.yaw = static_cast<double>(
        Hex::asInt(message.substr(2, 2) + message.substr(0, 2))
    ) / 10.0;

    attitude.pitch = static_cast<double>(
        Hex::asInt(message.substr(6, 2) + message.substr(4, 2))
    ) / 10.0;

    attitude.roll = static_cast<double>(
        Hex::asInt(message.substr(10, 2) + message.substr(8, 2))
    ) / 10.0;

    attitude.yawSpeed = static_cast<double>(
        Hex::asInt(message.substr(14, 2) + message.substr(12, 2))
    ) / 10.0;

    attitude.pitchSpeed = static_cast<double>(
        Hex::asInt(message.substr(14, 2) + message.substr(12, 2))
    ) / 10.0;

    attitude.rollSpeed = static_cast<double>(
        Hex::asInt(message.substr(22, 2) + message.substr(20, 2))
    ) / 10.0;

}

void SIYI::parseGimbalInfo(string message, uint16_t seq) {

    unique_lock<mutex> lock(stateMut);  

    MotionState &motion = state.motion;

    motion.seq = seq;
    motion.mode = Hex::asInt(message.substr(8, 2));

}

// void parseGimbalSpeed(string message, uint16_t seq)
// void parseGimbalCenter(string message, uint16_t seq)

void SIYI::parseZoom(string message, uint16_t seq) {

    unique_lock<mutex> lock(stateMut);  

    ZoomState &zoom = state.zoom;

    zoom.level = static_cast<double>(Hex::asInt(message.substr(2, 2) + message.substr(0, 2))) / 10.0;
    zoom.seq = seq;

}


// set functions

void SIYI::setGimbalRotation(double yaw, double pitch, double errorThreshold, double Kp) {

    if (yaw > 45.0 || yaw < - 45.0) {
        cout << "yaw outside bounds" << endl;
        return;
    }

    if (pitch > 25.0 || pitch < - 90.0) {
        cout << "pitch outside bounds" << endl;
        return;
    }

    while (true) {
        SIYI::requestGimbalAttitude();

        unique_lock<mutex> lock(stateMut);  

        if (state.attitude.seq != state.attitude.lastSeq) {
            SIYI::requestGimbalSpeed(0,0);
            continue;
        }

        // Update last seq
        state.attitude.lastSeq = state.attitude.seq; 

        double yawError = -yaw + state.attitude.yaw;
        double pitchError = pitch - state.attitude.pitch;

        if (fabs(yawError) <= errorThreshold && fabs(pitch) <= errorThreshold) {
            SIYI::requestGimbalSpeed(0,0);
            break;
        }

        SIYI::requestGimbalSpeed(
            max(min(100, static_cast<int>(Kp * yawError)), -100),
            max(min(100, static_cast<int>(Kp * pitchError)), -100)
        );

        this_thread::sleep_for(chrono::milliseconds(SET_GIMBAL_INTERVAL));

    }


}


