#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>
#include <mavsdk/log_callback.h>
#include "agent.h"
#include "detect.h"

#define PHYSICAL_PORT_PATH "serial:///dev/ttyTHS0"
#define SIM_PORT_PATH  "udp://:14540"

#define TEN_METERS_APPROX 0.0004

using std::this_thread::sleep_for;
using std::chrono::seconds;

int main()
{
    std::ofstream myfile;
    std::ofstream mavlog;
    myfile.open("../logs.txt");
    mavlog.open("../mavlog.txt");

    if(!myfile.is_open()){
        std::cout << "File open failed! Ending program." << std::endl;
        return 0;
    }

    if(!mavlog.is_open()){
        std::cout << "MavLog open failed! Ending program." << std::endl;
        return 0;
    }

    // Command to run the Python script
    /**const char* command = "python3 ../siyi_sdk-main/tests/test_zoom_2.py";

    // Execute the command
    int result = std::system(command);

    // Check if the command was executed successfully
    if (result == 0) {
        std::cout << "Successfuly started the script" << std::endl;
        myfile << "Successfuly started the script" << std::endl;
    } else {
        std::cout << "Failed to start script" << std::endl;
        myfile << "Failed to start script" << std::endl;
    }**/

    mavsdk::log::subscribe([&mavlog](mavsdk::log::Level level,   // message severity level
                          const std::string& message, // message text
                          const std::string& file,    // source file from which the message was sent
                          int line) {                 // line number in the source file
        
        mavlog << message << endl << file << line << endl << endl;

        // returning true from the callback disables printing the message to stdout
        return false;
        });


    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_any_connection(PHYSICAL_PORT_PATH);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        myfile << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        myfile << "Timed out waiting for system\n";
        return 1;
    }

   
    auto action = Action{system.value()};
    auto telemetry = Telemetry{system.value()};
    auto mission = mavsdk::Mission{system.value()};
    auto param = mavsdk::Param(system.value());

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        myfile << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }
    std::cout << "System is ready\n";
    myfile << "System is ready\n";

    /**const auto arm_result = action.arm();
    if (arm_result != Action::Result::Success) {
        std::cerr << "Arming failed: " << arm_result << '\n';
        return 1;
    }
    std::cout << "Armed\n";

    const auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success) {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    auto in_air_promise = std::promise<void>{};
    auto in_air_future = in_air_promise.get_future();
    Telemetry::LandedStateHandle handle = telemetry.subscribe_landed_state(
        [&telemetry, &in_air_promise, &handle](Telemetry::LandedState state) {
            if (state == Telemetry::LandedState::InAir) {
                std::cout << "Taking off has finished\n.";
                telemetry.unsubscribe_landed_state(handle);
                in_air_promise.set_value();
            }
        });

    in_air_future.wait_for(seconds(20));
    if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "Takeoff timed out.\n";
        return 1;
    }**/

    const auto set_velocity = param.set_param_float("MPC_XY_VEL_MAX", 20.0);
    if (set_velocity != mavsdk::Param::Result::Success) {
        std::cerr << "Velocity not set: " << set_velocity << '\n';
        myfile << "Velocity not set: " << set_velocity << '\n';
        return 1;
    }

    const auto set_z_velocity_down = param.set_param_float("MPC_Z_VEL_MAX_DN", 1.5);
    if (set_z_velocity_down != mavsdk::Param::Result::Success) {
        std::cerr << "Z Velocity not set: " << set_z_velocity_down << '\n';
        myfile << "Z Velocity not set: " << set_z_velocity_down << '\n';
        return 1;
    }

    const auto set_z_velocity_up = param.set_param_float("MPC_Z_VEL_MAX_UP", 3.0);
    if (set_z_velocity_up != mavsdk::Param::Result::Success) {
        std::cerr << "Z Velocity Up not set: " << set_z_velocity_up << '\n';
        myfile << "Z Velocity Up not set: " << set_z_velocity_up << '\n';
        return 1;
    }

    while(telemetry.flight_mode() != Telemetry::FlightMode::Mission) {
        std::cout << "Waiting for Mission\n";
        myfile << "Waiting for Mission" << endl;
        sleep_for(400ms);
    }


    Agent agent(
        action,
        mission,
        telemetry,
        myfile
    );

    const int LOOP_ALTITUDE = 12.5;
    const int SCAN_ALTITUDE = 12.5;

    agent.setLoopPoints({
        {telemetry.position().latitude_deg, telemetry.position().longitude_deg, LOOP_ALTITUDE},
        {telemetry.position().latitude_deg, telemetry.position().longitude_deg + TEN_METERS_APPROX, LOOP_ALTITUDE},
    });

    
    /**agent.setLoopPoints({
        {40.0930821, -83.1963847, LOOP_ALTITUDE},
        {40.0927715, -83.1963421, LOOP_ALTITUDE},
    });**/
    
    agent.setScanPoints({
        {telemetry.position().latitude_deg + TEN_METERS_APPROX, telemetry.position().longitude_deg + TEN_METERS_APPROX, SCAN_ALTITUDE},
        {telemetry.position().latitude_deg + TEN_METERS_APPROX, telemetry.position().longitude_deg, SCAN_ALTITUDE},
    });

   cout << "First scan point (" << telemetry.position().latitude_deg + TEN_METERS_APPROX << std::setprecision(8) << ", " << telemetry.position().longitude_deg + TEN_METERS_APPROX << std::setprecision(8) << ")" << std::endl; 


    /**agent.setScanPoints({
        {40.0926800, -83.1966677, SCAN_ALTITUDE},
        {40.0931109, -83.1967287, SCAN_ALTITUDE},
    });**/

    // load camera parameters
    agent.loadIntrinsics("../t_intrinsics.xml");
    myfile << "Loaded intrinsics" << endl; 
    agent.loadExtrinsics("../t_extrinsics.xml");
    myfile << "Load extrinsics" << endl;
    

    for (auto item : agent.lap_traj) {
        cout << "(" << item->latitude_deg << ", " << item->longitude_deg << ")" << std::endl;
        myfile << "(" << item->latitude_deg << ", " << item->longitude_deg << ")" << std::endl;
    }

    cout << "in main for agent" << endl;
    myfile << "in main for agent" << endl;

    agent.initTargets("../servoConfig.txt");


    cout << "actually starting the agent" << endl;
    myfile << "actually starting the agent" << endl;

    agent.start();


    // auto offboard_result = offboard.start();
    
    while (agent.isRunning())
        sleep_for(seconds(1));

    myfile << "closing the files..." << endl;

    myfile.close();
    mavlog.close();

    return 0;
}
