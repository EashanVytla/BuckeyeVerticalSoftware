// #include <bits/stdc++.h> 
#include <stdlib.h> 
#include <unistd.h> 
#include <string> 
#include <sys/types.h> 
#include <sys/socket.h> 
#include <arpa/inet.h> 
#include <netinet/in.h> 
#include <netinet/in.h> 
#include <SIYI.h>
#include <mutex> 
#include <thread>
#include <chrono>
#include <hex.h>

using namespace std;


void hello() {
    cout << "hello" << endl;
}


SIYI::SIYI(string ip, int port) {

    SIYI::port = port;
    SIYI::ip = ip;

};

bool SIYI::connect(int maxRetries) {

    active = true;

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

    while (counter < maxRetries) {
        
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

void SIYI::disconnect() {

    cout << "now disconnecting..." << endl;

    active = false;
    connected = false;

    for (auto&& thread : threads) {
        if (thread.joinable()) {
            thread.join();
        }
    }

    threads.clear();

    close(socketfd);

    // prevent reuse
    socketfd = -1;


    this_thread::sleep_for(chrono::seconds(5));

 
}


void SIYI::connectionLoop() {

    while (active) {
        SIYI::checkConnection();
        this_thread::sleep_for(chrono::milliseconds(CHECK_CONNECTION_INTERVAL));
    }

}

void SIYI::recvLoop() {


    while (active) {

        cout << "inside recvLoop" << endl;

        SIYI::bufferCallback();

    }

}

// TODO
void SIYI::bufferCallback() {
    cout << "bufferCallback" << endl;
}

// TODO
void SIYI::checkConnection() {
    cout << "checkConnection" << endl;
}

void SIYI::emit(string message) {
    vector<char> v = Hex::asVector(message);

    if (sendto(socketfd,  (const void*)v.data(), v.size(), 0, (sockaddr*)&servaddr, sizeof(servaddr)) < 0) {
        perror("error emitting message");
    }

}

void SIYI::emitAll() {
    while (queue.size() > 0) {
        emit(queue.dequeue());
    }
}



// Request functions