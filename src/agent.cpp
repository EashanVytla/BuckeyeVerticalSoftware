#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>

#include "agent.h"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>

using namespace mavsdk;
using namespace std;
using std::chrono::seconds;
using std::this_thread::sleep_for;

Agent::Agent(
    Action &newAction,
    Offboard &newOffboard,
    Telemetry &newTelemetry,
    Param &newParam,

    ofstream &logFile) : action(newAction), offboard(newOffboard), telemetry(newTelemetry), param(newParam), myfile(logFile)
{
    // First state is TAKEOFF
    state = State::TAKEOFF;
}

double Agent::distance(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude)
{
    double distance = abs(
        sqrt(pow(currLatitude - targetLatitude, 2) + pow(currLongitude - targetLongitude, 2)));
    return distance;
}

float Agent::yaw(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude)
{

    if ((targetLongitude - currLongitude) == 0)
        return 0.0;

    double position = (targetLatitude - currLatitude) / (targetLongitude - currLongitude);

    return (float)atan(position) * (180 / M_PI);
}

void Agent::updateState()
{

    switch (state)
    {
    
    // TAKEOFF
    // TODO: fix this later
    case State::TAKEOFF:
    {
        state = State::WAYPOINT;

        // heartbeat

        const Offboard::PositionGlobalYaw currentLocation{
            telemetry.position().latitude_deg,
            telemetry.position().longitude_deg,
            7.0f,
            0.0f};

        offboard.set_position_global(currentLocation);

        sleep_for(400ms);

    }
    break;

    // WAYPOINT
    case State::WAYPOINT:
    {

        // coordinate of current waypoint
        Coordinate target = waypointCtx.positions[waypointCtx.idx];

        double distance = Agent::distance(telemetry.position().latitude_deg, telemetry.position().longitude_deg, target.latitude, target.longitute);
        float yaw = Agent::yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, target.latitude, target.longitute);

        const Offboard::PositionGlobalYaw pgy{
            target.latitude,
            target.longitute,
            7.0f,
            yaw,
            Offboard::PositionGlobalYaw::AltitudeType::RelHome};

        offboard.set_position_global(pgy);

        // still too far
        if (distance > GEO_THRESHOLD) {
            sleep_for(400ms);
            break;
        }

        // TODO: IF TARGET OR SCAN IS CLOSED TO THIS POSITION, START THOSE MODES
        // if () {
        //     state = State::SCAN;
        // }

        // if () {
        //     state = State::DELIVERY;
        // }

        // Set next needed waypoint
        int nextWaypointIdx = (waypointCtx.idx + 1) % waypointCtx.positions.size(); 

        // completed a lap and back on the duplicated first waypoint
        if (nextWaypointIdx < waypointCtx.idx) {
            lapCounter++;
            deliveryCtx.requiresDrop = true; 

            // send the drone home after completing all laps
            if (lapCounter >= MAX_LAPS) {
                state = State::HOME;
            }
        }

        // this will not matter if the drone is set to HOME
        waypointCtx.idx = nextWaypointIdx;


        sleep_for(400ms);

    }
    break;

    // SCAN
    // Represents hitting all the waypoints for scanning
    case State::SCAN:
    {

        // coordinate of current scan waypoint
        Coordinate target = scanCtx.positions[scanCtx.idx];

        double distance = Agent::distance(telemetry.position().latitude_deg, telemetry.position().longitude_deg, target.latitude, target.longitute);
        float yaw = Agent::yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, target.latitude, target.longitute);

        const Offboard::PositionGlobalYaw pgy{
            target.latitude,
            target.longitute,
            7.0f,
            yaw,
            Offboard::PositionGlobalYaw::AltitudeType::RelHome};

        offboard.set_position_global(pgy);  


        // TODO: perform scan() here and update deliveryCtx from it

        // still too far
        if (distance > GEO_THRESHOLD) {
            sleep_for(400ms);
            break;
        }

        // now consider the next scanned dropzone
        int nextScanIdx = (scanCtx.idx + 1) % scanCtx.positions.size(); 

        // completed all the scan waypoints, next cyclical scan waypoint is the first 
        if (nextScanIdx < scanCtx.idx) {
            scanCtx.completedScanning = true;
            state = State::DELIVERY;
        }

        scanCtx.idx = nextScanIdx;

        sleep_for(400ms);
    }
    break;

    // DELIVERY
    case State::DELIVERY:
    {
        
    }
    break;

    // DROP
    case State::DROP:
    {
    }
    break;

    // OBJAVOID
    case State::OBJAVOID:
    {
    }
    break;

    // HOME
    case State::HOME:
    {
    }
    break;

    // STOP
    case State::STOP:
    {
    }
    break;

    // LAND
    case State::LAND:
    {
    }
    break;

    // DONE
    case State::DONE:
    {
    }
    break;

    }
}

void start()
{}