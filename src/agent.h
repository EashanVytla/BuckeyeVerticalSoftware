#pragma once

#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>
#include <string>
#include <vector>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>

#define PORT_PATH "serial:///dev/ttyTHS1"

using namespace mavsdk;
using namespace std;

enum class State
{
    TAKEOFF,
    WAYPOINT,
    SCAN, // the actual one-time scanning phase
    DELIVERY, // traveling to the payload drop zone
    DROP, // actually dropping the payload
    OBJAVOID,
    LOITER,
    HOME,
    STOP,
    LAND,
    DONE
};

struct Coordinate
{
    double longitute;
    double latitude;
    double altitude;
};

struct Context
{
    vector<Coordinate> positions;
    int idx;
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


class Agent
{
public:
    Agent(
        Action &action,
        Offboard &offboard,
        Telemetry &telemetry,
        Param &param,

        ofstream &myfile);
    double distance(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude);
    int findClosestPositionIdx(Coordinate test, vector<Coordinate> positions);
    float yaw(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude);
    void updateState();
    void sendHeartbeat(Offboard offboard, Telemetry telemetry);
    void start();
    void stop();
    // class vars

    // State data
    WaypointContext waypointCtx;
    ScanContext scanCtx;
    DeliveryContext deliveryCtx;

    // Current state
    State state;
    int lapCounter;

    int closestWaypointIdx; // closest waypoint to current delivery point

    // mavlink stuff
    Action &action;
    Offboard &offboard;
    Telemetry &telemetry;
    Param &param;

    ofstream &myfile;

    const double GEO_THRESHOLD = 0.00002;
    const int MAX_LAPS = 5;

};
