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

// NOTE: positions must be filled with atleast one element
int Agent::findClosestPositionIdx(Coordinate target, vector<Coordinate> positions)
{

    double minDistance = 10000.0;

    // Coordinate closest;

    int result;

    for (int i = 0; i < positions.size(); i++)
    {

        Coordinate current = positions.at(i);
        double currentDistance = Agent::distance(current.latitude, current.longitute, target.latitude, target.longitute);

        if (currentDistance > minDistance)
            continue;

        result = i;
        minDistance = currentDistance;
    }

    return result;
}

void Agent::sendHeartbeat(Offboard offboard, Telemetry telemetry)
{
    float currentLat = telemetry.position().latitude_deg;
    float currentLong = telemetry.position().longitude_deg;
    float currentAlt = telemetry.position().relative_altitude_m;

    float yaw = Agent::yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, currentLat, currentLong);

    const Offboard::PositionGlobalYaw pgy{
        currentLat,
        currentLong,
        currentAlt,
        yaw,
        Offboard::PositionGlobalYaw::AltitudeType::RelHome};

    offboard.set_position_global(pgy);
}

void Agent::updateState()
{

    switch (state)
    {

    // TAKEOFF
    // TODO: fix this later
    case State::TAKEOFF:
    {
        state = State::SCAN;

        // heartbeat
        Agent::sendHeartbeat(offboard, telemetry);

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
            target.altitude,
            yaw,
            Offboard::PositionGlobalYaw::AltitudeType::RelHome};

        offboard.set_position_global(pgy);

        // TODO: perform scan() here and update deliveryCtx from it

        // still too far
        if (distance > GEO_THRESHOLD)
        {
            sleep_for(400ms);
            break;
        }

        // now consider the next scanned dropzone
        int nextScanIdx = (scanCtx.idx + 1) % scanCtx.positions.size();

        // completed all the scan waypoints, next cyclical scan waypoint is the first
        if (nextScanIdx < scanCtx.idx)
        {
            scanCtx.completedScanning = true;

            // state = State::DELIVERY;
            state = State::LOITER;
        }

        scanCtx.idx = nextScanIdx;

        sleep_for(400ms);
    }
    break;

    // DELIVERY
    case State::DELIVERY:
    {

        // send heartbeat message
        Agent::sendHeartbeat(offboard, telemetry);

        // For testing
        state = State::DROP;

        sleep_for(400ms);
    }
    break;

    // DROP
    case State::DROP:
    {


        Agent::sendHeartbeat(offboard, telemetry);

        state = State::SCAN;

        sleep_for(400ms);

    }
    break;

    // DROP
    case State::LOITER:
    {
        Agent::sendHeartbeat(offboard, telemetry);

        sleep_for(400ms);
    }
    break;
    }
}

void start()
{
}