#pragma once
#ifndef AGENT_H
#define AGENT_H

#include <iostream>
#include <chrono>
#include <ctime>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>
#include <set>
#include <opencv2/opencv.hpp>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/mission/mission.h>

#include "detect.h"


using namespace mavsdk;
using namespace std;

struct Coordinate
{
    double latitude;
    double longitude;
    double altitude;
};

struct Context
{
    vector<Coordinate> positions;
    int idx = 0;
};

// holds the waypoints required in the lap
struct WaypointContext : Context
{
};

// holds the 4 positions needed to scan the purple rect. 
struct ScanContext : Context
{
    bool completedScanning = false; // to do the initial scan phase
};

// holds the dropzone positions
struct DeliveryContext : Context
{
    bool requiresDrop = false;      // needs a drop in the current loop
};


struct Servo
{
    int index;
    string className;
    double openPosition;
    double closePosition;
};


class Agent
{
public:
    Agent(
        Action &action,
        Mission &mission,
        Telemetry &telemetry,
        mavsdk::Param &param,

        ofstream &myfile);

    double distance(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude);

    void start();
    void stop();

    bool isRunning();
    
    void loop();
    void initTargets(string configPath);

    void safe_sleep(int msTime);

    ScanContext& getScanContext();
    WaypointContext& getWaypointContext();

    // class vars

    // State data
    WaypointContext waypointCtx;
    ScanContext scanCtx;
    DeliveryContext deliveryCtx;


    // Current state
    State state;
    
    // int lapCounter;
    // int closestWaypointIdx; // closest waypoint to current delivery point

    // mavlink stuff
    Action &action;
    Offboard &offboard;
    Telemetry &telemetry;
    mavsdk::Param &param;

    ofstream &myfile;

    vector<thread> threads;

    Detect detect;

    const double GEO_THRESHOLD = 0.00002;
    const int MAX_LAPS = 5;

    int candidateIdx = 0;

    set<int> detectedSet;
    set<int> targetSet;

    vector<Servo> servos;

};

#endif