#include "agent.h"

using namespace mavsdk;
using namespace std;

using std::chrono::seconds;
using std::this_thread::sleep_for;

std::atomic<bool> shouldRun(false);

double servoPosition = 0.0;
// int detectedClassIdx = 0;


Agent::Agent(
    Action &newAction,
    Mission &newMission,
    Telemetry &newTelemetry,
    ofstream &logFile) : action(newAction), mission(newMission), telemetry(newTelemetry), myfile(logFile)
{
    // First state is TAKEOFF
    state = State::TAKEOFF;
}

void Agent::updateState()
{

    switch (state)
    {

    // TAKEOFF
    // TODO: fix this later
    case State::TAKEOFF:
    {
        
        // TODO: set mission for INITIAL_LOOP

        Mission::MissionPlan mission_plan{};

        std::shared_ptr<Mission::MissionItem> currentPosition(new Mission::MissionItem());

        currentPosition->latitude_deg = telemetry.position().latitude_deg;
        currentPosition->longitude_deg = telemetry.position().longitude_deg;
        currentPosition->loiter_time_s = 1.0f; // loiter for 15 seconds
        currentPosition->is_fly_through = false;
        currentPosition->relative_altitude_m = lap_traj.at(0)->relative_altitude_m;
        currentPosition->speed_m_s = LAP_SPEED; // 10 meters per second for the speed

        mission_plan.mission_items.push_back(*currentPosition);

        for (const auto& item : lap_traj) {
            mission_plan.mission_items.push_back(*item);
        }

        mission.upload_mission(mission_plan);
        auto result = mission.start_mission();
    
        while (result != Mission::Result::Success) {
            std::cout << "INITIAL_LOOP Mission start failed (" << result << "), exiting." << '\n';
            myfile << "INITIAL_LOOP Mission start failed (" << result << "), exiting." << '\n';
            result = mission.start_mission();
            sleep_for(400ms);
        }


        state = State::INITIAL_LOOP;

        myfile << "Changed state to INITIAL_LOOP" << endl;
        cout << "Changed state to INITIAL_LOOP" << endl;


        sleep_for(400ms);
    }
    break;

    case State::INITIAL_LOOP:
    {
        auto completionCheckPair = mission.is_mission_finished();
        if (completionCheckPair.first != Mission::Result::Success) {
            myfile << "Failed to get mission status: " << std::endl;
            cout << "Failed to get mission status: " << std::endl;
            sleep_for(400ms);
            break;
        }

        if (!completionCheckPair.second) {
            myfile << "Mission is not complete yet: " << std::endl;
            cout << "INITIAL_LOOP Mission is not complete yet: " << std::endl;
            sleep_for(400ms);
            break;
        }


        cout << "INITIAL_LOOP Mission WAS COMPLETE: " << std::endl;
        myfile << "INITIAL_LOOP Mission WAS COMPLETE: " << std::endl;
        // TODO: set mission for SCAN

        Mission::MissionPlan mission_plan{};

        std::shared_ptr<Mission::MissionItem> currentPosition(new Mission::MissionItem());

        currentPosition->latitude_deg = telemetry.position().latitude_deg;
        currentPosition->longitude_deg = telemetry.position().longitude_deg;
        currentPosition->loiter_time_s = 1.0f; // loiter for 15 seconds
        currentPosition->is_fly_through = false;
        currentPosition->relative_altitude_m = scan_traj.at(0)->relative_altitude_m;
        currentPosition->speed_m_s = LAP_SPEED; // 10 meters per second for the speed

        mission_plan.mission_items.push_back(*currentPosition);
        mission_plan.mission_items.push_back(*scan_traj.at(0));

        mission.upload_mission(mission_plan);

        auto result = mission.start_mission();
    
        while (result != Mission::Result::Success) {
            std::cout << "TRAVELING_TO_SCAN Mission start failed (" << result << "), exiting." << '\n';
            myfile << "TRAVELING_TO_SCAN Mission start failed (" << result << "), exiting." << '\n';
            result = mission.start_mission();
            sleep_for(400ms);
        }

        state = State::TRAVELING_TO_SCAN;

    }
    break;

    // TRAVELING_TO_SCAN
    case State::TRAVELING_TO_SCAN:
    {

        auto completionCheckPair = mission.is_mission_finished();
        if (completionCheckPair.first != Mission::Result::Success) {
            myfile << "Failed to get mission status: " << std::endl;
            cout << "Failed to get mission status: " << std::endl;
            sleep_for(400ms);
            break;
        }

        if (!completionCheckPair.second) {
            myfile << "TRAVELING_TO_SCAN Mission is not complete yet: " << std::endl;
            cout << "TRAVELING_TO_SCAN Mission is not complete yet: " << std::endl;
            sleep_for(400ms);
            break;
        }


        Mission::MissionPlan mission_plan{};

        for (const auto& item : scan_traj) {
            mission_plan.mission_items.push_back(*item);
        }

        mission.upload_mission(mission_plan);
        auto result = mission.start_mission();

        while (result != Mission::Result::Success) {
            std::cout << "SCAN Mission start failed (" << result << "), exiting." << '\n';
            myfile << "SCAN Mission start failed (" << result << "), exiting." << '\n';
            result = mission.start_mission();
            sleep_for(400ms);
        }

        state = State::SCAN;

    }
    break;

    // SCAN
    // Represents hitting all the waypoints for scanning
    case State::SCAN:
    {
        auto completionCheckPair = mission.is_mission_finished();
        if (completionCheckPair.first != Mission::Result::Success) {
            myfile << "Failed to get mission status: " << std::endl;
            cout << "SCAN Failed to get mission status: " << std::endl;
            sleep_for(400ms);
            break;
        }

        if (completionCheckPair.second) {

            if (detectedPositions.size() <= 0) {
                state = State::LOITER;
                break; 
            }

            // TODO: set mission for INITIAL_DELIVERY
            std::vector<std::shared_ptr<Mission::MissionItem>> connector_traj;

            std::shared_ptr<Mission::MissionItem> currentPosition(new Mission::MissionItem());

            currentPosition->latitude_deg = telemetry.position().latitude_deg;
            currentPosition->longitude_deg = telemetry.position().longitude_deg;
            currentPosition->loiter_time_s = 1.0f; // loiter for 15 seconds
            currentPosition->is_fly_through = false;
            currentPosition->relative_altitude_m = scan_traj.at(0)->relative_altitude_m;
            currentPosition->speed_m_s = SCAN_SPEED; // 10 meters per second for the speed



            // connector_traj.push(make_mission_item(telemetry.position().latitude_deg,
            //           telemetry.position().longitude_deg, altitude, speed, false,
            //           0.0f, 0.0f,
            //           MissionItem::CameraAction::NONE))

            connector_traj.push_back(currentPosition);
            // connector_traj.push_back(currentPosition);
            connector_traj.push_back(detectedPositions.top());

            Mission::MissionPlan mission_plan{};

            for (const auto& item : connector_traj) {
                mission_plan.mission_items.push_back(*item);
            }

            Mission::Result result = mission.upload_mission(mission_plan);

            if (result != Mission::Result::Success) {
                std::cout << "Mission upload failed (" << result << "), exiting." << std::endl;
                myfile << "Mission upload failed (" << result << "), exiting." << std::endl;
                return;
            }

            state = State::INITIAL_DELIVERY;
            break;
        }

        myfile << "Mission is not complete yet: " << std::endl;
        cout << "SCAN Mission is not complete yet: " << std::endl;
        
        // get only candidate idx to perform initial checks

        candidateIdx = detect.getDetectedClassIdx();


        if (candidateIdx <= -1) {
            sleep_for(400ms);
            break;
        }

        myfile << "Candidate IDX: " << candidateIdx << endl << endl;
        cout << "Candidate IDX: " << candidateIdx << endl << endl;

        if (targetSet.count(candidateIdx) <= detectedSet.count(candidateIdx)) {
            sleep_for(400ms);
            break;
        }

        // pause the mission

        Mission::Result result = mission.pause_mission();

        while (result != Mission::Result::Success) {
            myfile << "Failed to pause mission" << std::endl;
            cout << "Failed to pause mission" << std::endl;
            result = mission.pause_mission();
            sleep_for(400ms);
        }

        
        // becoming still before looking at bounding box
        sleep_for(1000ms);

        // double check to make sure we still see the target

        detect.lockInference();

        int doubleCheckCandidateIdx = detect.getDetectedClassIdxUnsafe();
        currentDropTargetPos = detect.getDetectedBBoxUnsafe();

        detect.unlockInference();


        // only add the new target if double check has same candidate idx
        if (doubleCheckCandidateIdx == candidateIdx) {


            // perform localization
            double u = currentDropTargetPos.x + currentDropTargetPos.width / 2;
            double v = currentDropTargetPos.y + currentDropTargetPos.height / 2;

            double originX = 640;
            double originY = 360;

            const double INCHES_PER_METER = 39.3701;
            const double METERS_PER_INCH = 1 / INCHES_PER_METER;
            const double INCHES_PER_CHECKER = 0.875;
            const double METERS_PER_CHECKER = INCHES_PER_CHECKER * METERS_PER_INCH;
            const double CHECKERS_PER_INCH = 1 / INCHES_PER_CHECKER;
            const double INCHES_OFFSET = 58.125;

            const double zConst = (scan_traj.at(0)->relative_altitude_m * INCHES_PER_METER - INCHES_OFFSET) * CHECKERS_PER_INCH;

            Mat alignedPoint = alignedProject(zConst, u, v, originX, originY, inverseCameraMatrix, inverseRotationMatrix, translationVector);

            // cout << alignedPoint << endl;

            double offset_x = alignedPoint.at<double>(0) * METERS_PER_CHECKER;
            double offset_y = alignedPoint.at<double>(1) * METERS_PER_CHECKER;

            double lat = telemetry.position().latitude_deg;
            double lon = telemetry.position().longitude_deg;

            double heading = telemetry.heading().heading_deg;

            std::pair<double,double> latLongCalc = offset_to_latlon(lat, lon, heading, offset_x, offset_y);

            double targetLatitude = latLongCalc.first;
            double targetLongitude = latLongCalc.second;


            // create mission item for target
            std::shared_ptr<Mission::MissionItem> targetPosition(new Mission::MissionItem());

            targetPosition->latitude_deg = targetLatitude;
            targetPosition->longitude_deg = targetLongitude;
            targetPosition->loiter_time_s = 3.0f; // loiter for 3 seconds
            targetPosition->is_fly_through = false;
            
            detectedPositions.push(targetPosition);
            detectedClassNumbers.push(candidateIdx);

            // make sure target doesn't get detected again
            detectedSet.insert(candidateIdx);

        }

        // restart the mission

        result = mission.start_mission();

        while (result != Mission::Result::Success) {
            myfile << "Failed to restart mission" << std::endl;
            result = mission.start_mission();
            sleep_for(400ms);
        }


    }
    break;

    // INITIAL_DELIVERY
    // Represents hitting all the waypoints for scanning
    case State::INITIAL_DELIVERY:
    {
        auto completionCheckPair = mission.is_mission_finished();
        if (completionCheckPair.first != Mission::Result::Success) {
            myfile << "Failed to get mission status: " << std::endl;
            cout << "INITIAL_DELIVERY Failed to get mission status: " << std::endl;
            sleep_for(400ms);
            break;
        }

        if (!completionCheckPair.second) {
            myfile << "Mission is not complete yet: " << std::endl;
            cout << "INITIAL_DELIVERY Mission is not complete yet: " << std::endl;
            
            sleep_for(400ms);
            break;
        }

        cout << "Completed INITIAL_DELIVERY Mission" << std::endl;
        myfile << "Completed INITIAL_DELIVERY Mission" << std::endl;
            
        // TODO: set mission for DROP
        sleep_for(2000ms);
        state = State::DROP;

    }
    break;

    // DROP
    case State::DROP:
    {




        string detectedClassName = detect.getClassNames().at(detectedClassNumbers.top());

	    // std::cout << "detected: " << detectedClassName << std::endl;
        myfile << "DROPPING FOR: " << detectedClassName << std::endl;


        for (int i = 0; i < servos.size(); i++)
        {
            if (detectedClassName == servos.at(i).className)
            {
                // std::cout << "Setting actuator " << detectedClassName << " : " << i << std::endl;
                myfile << "Setting actuator " << detectedClassName << " : " << i << std::endl;
                
                action.set_actuator(servos.at(i).index, servos.at(i).openPosition);
                break;
            }
        }

        // pop stacks
        detectedPositions.pop();
        detectedClassNumbers.pop();

        myfile << "Changed state to ROUTING" << endl;
        state = State::ROUTING;

        sleep_for(400ms);
    }
    break;

    case State::ROUTING:
    {
        
        // IF: max loops is done, then loiter ELSE

        // TODO: add the regular loop
        // TODO: add a target if it exists
        if(detectedPositions.size() > 0){
            std::vector<std::shared_ptr<Mission::MissionItem>> new_lap_traj(lap_traj);
            new_lap_traj.push_back(detectedPositions.top());

            Mission::MissionPlan mission_plan{};

            for (const auto& item : new_lap_traj) {
                mission_plan.mission_items.push_back(*item);
            }

            mission.upload_mission(mission_plan);

            state = State::TARGET_LOOP;
        }else{
            state = State::LOITER;
        }
    }
    break;

    case State::TARGET_LOOP:
    {

        auto completionCheckPair = mission.is_mission_finished();
        if (completionCheckPair.first != Mission::Result::Success) {
            myfile << "Failed to get mission status: " << std::endl;
            sleep_for(400ms);
            break;
        }

        if (completionCheckPair.second) {
            sleep_for(2000ms);
            state = State::DROP;
            break;
        }
           
    }
    break;


    // LOITER
    case State::LOITER:
    {
        // TODO: add loitering code
        cout << "Now loitering" << endl;
        myfile << "Now loitering" << endl;
        sleep_for(1000ms);

    }
    break;
    }
}



void Agent::initTargets(string servoConfig)
{

    // std::ifstream file(configFile);
    std::ifstream file(servoConfig);

    if (!file.is_open())
    {
        std::cerr << "Failed to open file." << std::endl;
        myfile << "FAILED TO OPEN servoConfig FILE" << std::endl;
        return;
    }

    // cout << "Made it to config file" << endl;
    myfile << "OPENED servoConfig FILE" << std::endl;

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
            std::cerr << "Invalid format for servoConfig.txt: " << line << std::endl;
            myfile << "Invalid format for servoConfig.txt: " << line << std::endl;
            break; // changed this to be a break
        }

        servos.push_back(servo);
    }

    file.close();

    // Printing the config entries
    // std::cout << "Config entries extracted from the file:" << std::endl;
    myfile << "Config entries extracted from the file:" << std::endl;


    for (const auto &servo : servos)
    {
        std::cout << "servoIndex: " << servo.index << ", Class name: " << servo.className << ", Open Position: " << servo.openPosition << ", Close Position " << servo.closePosition << std::endl;
        myfile << "servoIndex: " << servo.index << ", Class name: " << servo.className << ", Open Position: " << servo.openPosition << ", Close Position " << servo.closePosition << std::endl;
    
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
            myfile << "[Time elapsed: " << elapsed_seconds.count() << " seconds]" << std::endl;
            Agent::updateState();
            myfile << std::endl << std::endl;

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
    
    // detect.model_on("4xZoomCrop.mp4");

    shouldRun = true;

    // add the loop thread
    thread t_loop(&Agent::loop, this);
    threads.push_back(std::move(t_loop));

    // std::cout << "inside AGENT:start function" << std::endl;
    myfile << "ADDED LOOP THREAD INSIDE AGENT::start" << std::endl;

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

void Agent::setLoopPoints(std::vector<Coordinate> coords) {
    lap_traj.clear();

    for (int i = 0; i < coords.size(); i++) {

        std::shared_ptr<Mission::MissionItem> item(new Mission::MissionItem());

        item->latitude_deg = coords.at(i).latitude;
        item->longitude_deg = coords.at(i).longitude;
        item->relative_altitude_m = coords.at(i).altitude;
        item->speed_m_s = LAP_SPEED; // 10 meters per second for the speed
        item->loiter_time_s = 1.0f;
        item->is_fly_through = false;

        lap_traj.push_back(item);
    }
}

void Agent::setScanPoints(std::vector<Coordinate> coords) {
    scan_traj.clear();

    for (int i = 0; i < coords.size(); i++) {

        std::shared_ptr<Mission::MissionItem> item(new Mission::MissionItem());

        item->latitude_deg = coords.at(i).latitude;
        item->longitude_deg = coords.at(i).longitude;
        item->relative_altitude_m = coords.at(i).altitude;
        item->is_fly_through = false;
        item->loiter_time_s = 1.0f;
        item->speed_m_s = SCAN_SPEED;

        scan_traj.push_back(item);

    }
}

bool Agent::isRunning() {
    return shouldRun;
}


void Agent::loadIntrinsics(string file) {

    cv::FileStorage fi(file, cv::FileStorage::READ);

    fi["cameraMatrix"] >> cameraMatrix;
    fi["distortion"] >> distortionCoeffs;

    inverseCameraMatrix = cameraMatrix.inv();

    fi.release();

}

void Agent::loadExtrinsics(string file) {

    cv::FileStorage fe(file, cv::FileStorage::READ);

    fe["rotationMatrix"] >> rotationMatrix;
    fe["translationVector"] >> translationVector;

    inverseRotationMatrix = rotationMatrix.inv();

    fe.release();

}

