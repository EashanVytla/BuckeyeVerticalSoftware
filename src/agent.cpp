#include "agent.h"

using namespace mavsdk;
using namespace std;

using std::chrono::seconds;
using std::this_thread::sleep_for;

std::atomic<bool> shouldRun(false);

double servoPosition = 0.0;
// int detectedClassIdx = 0;



int servoCounter = 0;


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
        double currentDistance = Agent::distance(current.latitude, current.longitude, target.latitude, target.longitude);

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
    float currentYaw = telemetry.heading().heading_deg;

    //float yaw = Agent::yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, currentLat, currentLong);

    const Offboard::PositionGlobalYaw pgy{
        currentLat,
        currentLong,
        currentAlt,
        currentYaw,
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

        cout << "MADE IT TO SCAN, target:" << endl;
	

        // coordinate of current scan waypoint
        Coordinate target = scanCtx.positions[scanCtx.idx];

        cout << target.latitude << ", " << target.longitude << endl;
        cout << endl;

	    myfile << "TARGET: " << endl;
        myfile << target.latitude << ", " << target.longitude << endl;
        myfile << endl;
	
        double distance = Agent::distance(telemetry.position().latitude_deg, telemetry.position().longitude_deg, target.latitude, target.longitude);
        float yaw = Agent::yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, target.latitude, target.longitude);

        myfile << "Yaw: " << yaw << endl << endl;
        myfile << "Distance: " << distance << endl << endl;
        myfile << "Current: " << telemetry.position().latitude_deg << ", " << telemetry.position().longitude_deg << endl << endl;

        const Offboard::PositionGlobalYaw pgy{
            target.latitude,
            target.longitude,
            target.altitude,
            yaw,
            Offboard::PositionGlobalYaw::AltitudeType::RelHome};

        offboard.set_position_global(pgy);

        // TODO: perform scan() here and update deliveryCtx from it

        candidateIdx = detect.getDetectedClassIdx();

        if (candidateIdx > -1) {
		    myfile << "Candidate IDX: " << candidateIdx << endl << endl;
            // if hasn't already been scanned
            if (targetSet.count(candidateIdx) > detectedSet.count(candidateIdx)) {
                
                state = State::DROP;
                cout << "Changed state to DROP" << endl;
                myfile << "Changed state to DROP" << endl;

                sleep_for(400ms);
                
                // don't allow state to get overwritten
                break;
            }
        }

        cout << "distance: " << distance << endl;
        cout << "current idx " << scanCtx.idx << endl;
        cout << endl;

        // still too far
        if (distance > GEO_THRESHOLD)
        {
            sleep_for(400ms);
            break;
        }


        cout << "MADE IT PAST THE DISTANCE STUFF..." << endl;
        myfile << "MADE IT PAST THE DISTANCE STUFF..." << endl;

        // REACHED THE RIGHT SCANPOINT
        // now consider the next scanned dropzone
        int nextScanIdx = (scanCtx.idx + 1) % scanCtx.positions.size();
        myfile << "Next Scan IDX: " << nextScanIdx << endl;

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

        string detectedClassName = detect.getClassNames().at(candidateIdx);

	    std::cout << "detected: " << detectedClassName << std::endl;

        for (int i = 0; i < servos.size(); i++)
        {
            if (detectedClassName == servos.at(i).className)
            {
		    std::cout << "Setting actuator " << detectedClassName << " : " << i << std::endl;
		    myfile << "Setting actuator " << detectedClassName << " : " << i << std::endl;
                action.set_actuator(servos.at(i).index, servos.at(i).openPosition);
                break;
            }
        }

        detectedSet.insert(candidateIdx);
        // detect.setDetectedState(false);

        Agent::sendHeartbeat();

        state = State::SCAN;
        // myfile << "Changed state to SCAN" << endl;

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



void Agent::initTargets(string configPath)
{

    // std::ifstream file(configFile);
    std::ifstream file(configPath);

    if (!file.is_open())
    {
        std::cerr << "Failed to open file." << std::endl;
        return;
    }

    cout << "Made it to config file" << endl;

    // clear servos vector before repopulating it
    servos.clear();

    detectedSet.clear();
    targetSet.clear();

    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        Servo servo;

        // TODO: add this to the jetson
        if (!(iss >> servo.index >> servo.className >> servo.openPosition >> servo.closePosition))
        {
            std::cerr << "Invalid format: " << line << std::endl;
            myfile << "Invalid format: " << line << std::endl;
            break; // changed this to be a break
        }

        servos.push_back(servo);
    }

    file.close();

    // Printing the config entries
    std::cout << "Config entries extracted from the file:" << std::endl;
    for (const auto &servo : servos)
    {
        std::cout << "servoIndex: " << servo.index << ", Class name: " << servo.className << ", Open Position: " << servo.openPosition << ", Close Position " << servo.closePosition << std::endl;
    }

    // adding to targetSet
    for (int i = 0; i < servos.size(); i++)
        targetSet.insert(detect.getClassIdx(servos.at(i).className));

}



void Agent::loop() {
    auto startTime = std::chrono::system_clock::now();
    while (shouldRun)
    {
        try{
            // Get the end time
            auto current = std::chrono::system_clock::now();

            // Calculate the duration
            std::chrono::duration<double> elapsed_seconds = current - startTime;

            // Output the duration
            myfile << "Time elapsed: " << elapsed_seconds.count() << " seconds" << std::endl;
            Agent::updateState();
        } catch(const std::exception& e){
            myfile << "ERROR: " << e.what() << endl;
        }
    }

    myfile << "Ended shouldRun loop" << endl;
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

    std::cout << "inside AGENT:start function" << std::endl;

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

ScanContext& Agent::getScanContext() {
    return scanCtx;
}

WaypointContext& Agent::getWaypointContext() {
    return waypointCtx;
}

bool Agent::isRunning() {
    return shouldRun;
}
