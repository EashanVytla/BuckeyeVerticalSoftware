#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>
#include <set>
#include "opencv2/opencv.hpp"

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>

using namespace mavsdk;
using namespace std;
using std::chrono::seconds;

// #include "detect.h"
#include "agent.h"

// using std::this_thread::std::this_thread::sleep_for;

#define PORT_PATH "serial:///dev/ttyTHS1"


int main()
{
    std::ofstream myfile("Logs.txt");
    if(!myfile.is_open()){
      cout << "File open failed! Ending program." << endl;
      return 0;
   }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_any_connection(PORT_PATH);

    if(connection_result != ConnectionResult::Success){
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
    auto offboard = Offboard{system.value()};
    auto telemetry = Telemetry{system.value()};
    auto param = mavsdk::Param{system.value()};


    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        myfile << "Waiting for system to be ready\n";
        std::this_thread::sleep_for(1000ms);
    }
    std::cout << "System is ready\n";
    myfile << "System is ready\n";

    const auto set_velocity = param.set_param_float("MPC_XY_VEL_MAX", 5.0);
    if (set_velocity != mavsdk::Param::Result::Success) {
        std::cerr << "Velocity not set: " << set_velocity << '\n';
        myfile << "Velocity not set: " << set_velocity << '\n';
        return 1;
    }

    while(telemetry.flight_mode() != Telemetry::FlightMode::Offboard) {
        std::cout << "Waiting for Offboard\n";
        myfile << "Waiting for Offboard\n";
        const Offboard::PositionGlobalYaw currLocation{
                telemetry.position().latitude_deg,
                telemetry.position().longitude_deg,
                7.0f,
                0.0f};
        offboard.set_position_global(currLocation);
        std::this_thread::sleep_for(400ms);
    }

    auto in_air_promise = std::promise<void>{};
    auto in_air_future = in_air_promise.get_future();
    Telemetry::LandedStateHandle handle = telemetry.subscribe_landed_state(
        [&telemetry, &in_air_promise, &handle](Telemetry::LandedState state) {
            if (state == Telemetry::LandedState::InAir) {
                telemetry.unsubscribe_landed_state(handle);
                in_air_promise.set_value();
            }
        });
    


    std::ofstream logFile("Logs.txt");
    if(!logFile.is_open()){
      cout << "File open failed! Ending program." << endl;
      return 0;
   }

    Agent agent(
        action,
        offboard,
        telemetry,
        param,
        logFile
    );

        // Agent(
        // Action &action,
        // Offboard &offboard,
        // Telemetry &telemetry,
        // mavsdk::Param &pa    Agent(
        // Action &action,
        // Offboard &offboard,
        // Telemetry &telemetry,
        // mavsdk::Param &param,

        // ofstream &myfile);ram,

        // ofstream &myfile);


    agent.scanCtx.positions = {
        {40.0979633, -83.1982978, 24.384},
        {40.0979265, -83.1985653, 24.384},
        {40.0978664, -83.1987608, 24.384},
        {40.0978334, -83.1989082, 24.384},
        {40.0978456, -83.1990924, 24.384},
        {40.0978456, -83.1990924, 24.384},
    }; 


    agent.start();

    // telemetry.subscribe_position([&](Telemetry::Position position) {

    //     if (telemetry.flight_mode()  == Telemetry::FlightMode::Offboard) {
    //         agent.start();
    //         // start();
    //     } 
    // });

    


}