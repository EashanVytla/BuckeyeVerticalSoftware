#include "agent.h"

using namespace mavsdk;
using namespace std;

using std::chrono::seconds;
using std::this_thread::sleep_for;


std::atomic<bool> shouldRun(false);


struct ServoEntry
{
    int index;
    string className;
    double position;
};

double servoPosition = 0.0;
// int detectedClassIdx = 0;
set<int> detectedSet;

vector<ServoEntry> servos;

Agent::Agent(
    Action &newAction,
    Offboard &newOffboard,
    Telemetry &newTelemetry,
    mavsdk::Param &newParam,
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

void Agent::sendHeartbeat()
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
        myfile << "Changed state to SCAN" << endl;

        // heartbeat
        Agent::sendHeartbeat();

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

        if (detect.getDetectedState())
        {
            if (detectedSet.count(detect.getDetectedClassIdx()) == 0)
            {
                // only for testing
                state = State::DROP;
                myfile << "Changed state to DROP" << endl;

                // reset flag for detecting object

                sleep_for(400ms);
                break;
            }
        }

        // detectedSet

        // still too far
        if (distance > GEO_THRESHOLD)
        {
            sleep_for(400ms);
            break;
        }

        // REACHED THE RIGHT SCANPOINT
        // now consider the next scanned dropzone
        int nextScanIdx = (scanCtx.idx + 1) % scanCtx.positions.size();

        // completed all the scan waypoints, next cyclical scan waypoint is the first
        if (nextScanIdx < scanCtx.idx)
        {
            scanCtx.completedScanning = true;

            // state = State::DELIVERY;
            state = State::LOITER;
            myfile << "Changed state to LOITER" << endl;
        }

        scanCtx.idx = nextScanIdx;

        sleep_for(400ms);
    }
    break;

    // // DELIVERY
    // case State::DELIVERY:
    // {

    //     // send heartbeat message
    //     Agent::sendHeartbeat(offboard, telemetry);

    //     // For testing
    //     state = State::DROP;

    //     sleep_for(400ms);
    // }
    // break;

    // DROP
    case State::DROP:
    {

        string detectedClassName = detect.getClassNames().at(detect.getDetectedClassIdx());

        for (int i = 0; i < servos.size(); i++)
        {
            if (detectedClassName == servos.at(i).className)
            {
                action.set_actuator(servos.at(i).index, servos.at(i).position);
                break;
            }
        }

        // update set
        detectedSet.insert(detect.getDetectedClassIdx());
        detect.setDetectedState(false);

        Agent::sendHeartbeat();

        state = State::SCAN;
        myfile << "Changed state to SCAN" << endl;

        sleep_for(400ms);
    }
    break;

    // DROP
    case State::LOITER:
    {
        Agent::sendHeartbeat();

        sleep_for(400ms);
    }
    break;
    }
}

void Agent::initServos(string configFile)
{

    std::ifstream file(configFile);

    if (!file.is_open())
    {
        std::cerr << "Failed to open file." << std::endl;
        return;
    }

    // clear servos vector before repopulating it
    servos.clear();


    // std::vector<ServoEntry> configEntries;
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        ServoEntry servo;

        if (!(iss >> servo.index >> servo.className >> servo.position))
        {
            std::cerr << "Invalid format: " << line << std::endl;
        }
    }

    file.close();

    // Printing the config entries
    std::cout << "Config entries extracted from the file:" << std::endl;
    for (const auto &servo : servos)
    {
        std::cout << "servoIndex: " << servo.index << ", Class name: " << servo.className << ", Position: " << servo.position << std::endl;
    }
}

void Agent::loop() {

    while (shouldRun)
    {
        Agent::updateState();
    }

}


void Agent::start()
{

    if (shouldRun) 
        return;

    detect.model_on();

    shouldRun = true;

    // add the loop thread
    thread t_loop(&Agent::loop, this);
    threads.push_back(std::move(t_loop));

    return;
}

void Agent::stop() {

    shouldRun = false;

    for (auto&& t : threads) {
        if (t.joinable()) {
            t.join();
        }

    }

    detect.model_off();

}