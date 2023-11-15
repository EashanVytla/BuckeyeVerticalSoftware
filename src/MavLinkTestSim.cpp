#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/mission/mission.h>
#include <iostream>
#include <mavsdk/plugins/param/param.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <thread>
#include <mavsdk/plugins/action/action.h>
#include <fstream>

//Fix these
#define PORT_PATH "udp://localhost:14540"
#define STARTUP_PARAM "-"
#define STARTUP_PARAM_THRESH 5

using namespace mavsdk;

int main(){
    std::ofstream myfile;
    myfile.open ("Logs.txt");

    if(!myfile.is_open()){
        std::cout << "File open failed! Ending program." << std::endl;
        return 0;
    }

    std::cout << "File success!" << std::endl;
    myfile << "File successfully opened!" << '\n';

    //Create mavsdk object on stack
    Mavsdk mavsdk;
    
    //Connect to the Pixhawk
    ConnectionResult connection_result = mavsdk.add_any_connection(PORT_PATH);
    
    //Log connection failure
    if (connection_result != ConnectionResult::Success) {
        std::cout << "Adding connection failed: " << connection_result << '\n';
        return 0;
    }

    while (mavsdk.systems().size() == 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "Connected!" << '\n';

    // Get the list of systems
    auto systems = mavsdk.systems();

    // Check if there is at least one system
    if (systems.empty()) {
        std::cout << "No MAVLink system found!";
        return 0;
    }
        
    std::shared_ptr<mavsdk::System> system = systems[0];

    // Instantiate plugins.
    auto telemetry = Telemetry{system};
    auto action = Action{system};

    // We want to listen to the altitude of the drone at 1 Hz.
    const auto set_rate_result = telemetry.set_rate_position(1.0);
    if (set_rate_result != Telemetry::Result::Success) {
        std::cerr << "Setting rate failed: " << set_rate_result << '\n';
        return 1;
    }

    // Check until vehicle is ready to arm
    while (telemetry.health_all_ok() != true) {
        std::cout << "Vehicle is getting ready to arm\n";
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    // Arm vehicle
    std::cout << "Arming...\n";
    const Action::Result arm_result = action.arm();

    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }

    // Take off
    std::cout << "Taking off...\n";
    const Action::Result takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    // Let it hover for a bit before landing again.
    std::this_thread::sleep_for(std::chrono::seconds(10));

    auto mission = Mission{system};

    std::cout << "Creating waypoint vector..." << std::endl;

    //Create a vector to store all waypoints
    std::vector<std::shared_ptr<Mission::MissionItem>> mission_items;

    std::cout << "Creating first waypoint..." << std::endl;

    //Waypoint 1
    std::shared_ptr<Mission::MissionItem> waypoint1 = std::make_shared<Mission::MissionItem>();
    waypoint1->latitude_deg = 47.3978233; //Get these from QGC
    waypoint1->longitude_deg = 8.5455855;
    waypoint1->speed_m_s = 10;

    std::cout << "Creating second waypoint..." << std::endl;

    //Waypoint 2
    std::shared_ptr<Mission::MissionItem> waypoint2 = std::make_shared<Mission::MissionItem>();
    waypoint2->latitude_deg = 47.3975233;
    waypoint2->longitude_deg = 8.5458855;
    waypoint2->speed_m_s = 0;

    std::cout << "Creating third waypoint..." << std::endl;

    //Waypoint 3
    std::shared_ptr<Mission::MissionItem> waypoint3 = std::make_shared<Mission::MissionItem>();
    waypoint3->latitude_deg = 47.3975233;
    waypoint3->longitude_deg = 8.5455855;
    waypoint3->speed_m_s = 0;

    std::cout << "Creating fourth waypoint..." << std::endl;

    //Waypoint 4
    std::shared_ptr<Mission::MissionItem> waypoint4 = std::make_shared<Mission::MissionItem>();
    waypoint4->latitude_deg = 47.3977233;
    waypoint4->longitude_deg = 8.5455855;
    waypoint4->speed_m_s = 0;

    std::cout << "Adding all waypoint to vector..." << std::endl;

    mission_items.push_back(waypoint1); 
    mission_items.push_back(waypoint2); 
    mission_items.push_back(waypoint3); 
    mission_items.push_back(waypoint4); 

    std::cout << "Uploading mission..." << '\n';
    Mission::MissionPlan mission_plan{};

    for (const auto& item : mission_items) {
        mission_plan.mission_items.push_back(*item);
    }

    Mission::Result result = mission.upload_mission(
        mission_plan);

    if (result != Mission::Result::Success) {
        std::cout << "Mission upload failed (" << result << "), exiting." << '\n';
        return 1;
    }

    std::cout << "Mission uploaded." << '\n';

    result = mission.start_mission();

    if (result != Mission::Result::Success) {
        std::cout << "Mission start failed (" << result << "), exiting." << '\n';
        return 1;
    }
    std::cout << "Started mission." << '\n';

    mission.subscribe_mission_progress([](Mission::MissionProgress mission_progress){
        std::cout << "Mission status update: " << mission_progress.current << " / " << mission_progress.total << '\n';
    });

    myfile.close();
}