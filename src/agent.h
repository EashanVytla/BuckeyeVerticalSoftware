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
#include <stack>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mission/mission.h>

#include "detect.h"
#include "localization.h"


using namespace mavsdk;
using namespace std;

struct Coordinate
{
    double latitude;
    double longitude;
    double altitude;
};


enum class State
{
    TAKEOFF,
    WAYPOINT,
    INITIAL_LOOP,
    TARGET_LOOP,
    TRAVELING_TO_SCAN,
    SCAN, // the actual one-time scanning phase
    // DELIVERY, // traveling to the payload drop zone
    INITIAL_DELIVERY,
    ROUTING,
    DROP, // actually dropping the payload
    OBJAVOID,
    LOITER,
    HOME,
    STOP,
    LAND,
    DONE
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
        ofstream &myfile);

    void start();
    void stop();

    bool isRunning();
    
    void loop();
    void initTargets(string configPath);

    void setLoopPoints(std::vector<Coordinate> coords);
    void setScanPoints(std::vector<Coordinate> coords);

    void updateState();

    // Current state
    State state;
    
    // mavlink stuff
    Action &action;
    Mission &mission;
    Telemetry &telemetry;

    ofstream &myfile;

    vector<thread> threads;

    Detect detect;

    const double GEO_THRESHOLD = 0.00002;
    const int MAX_LAPS = 5;

    // const static float AGENT_ALTITUDE = 12.25;
    const float LAP_SPEED = 5.0;
    const float SCAN_SPEED = 5.0;

    int candidateIdx = 0;
    cv::Rect currentDropTargetPos = cv::Rect{0, 0, 0, 0};

    set<int> detectedSet;
    set<int> targetSet;

    vector<Servo> servos;
    std::vector<std::shared_ptr<Mission::MissionItem>> lap_traj;
    std::vector<std::shared_ptr<Mission::MissionItem>> scan_traj;

    std::stack<std::shared_ptr<Mission::MissionItem>> detectedPositions;
    std::stack<int> detectedClassNumbers;


    // camera parameters
    cv::Mat cameraMatrix;
    cv::Mat distortionCoeffs;
    cv::Mat rotationMatrix;
    cv::Mat translationVector;
    cv::Mat inverseCameraMatrix;
    cv::Mat inverseRotationMatrix;

    void loadIntrinsics(string file);
    void loadExtrinsics(string file);

};

#endif
