#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>

#include "agent.h"
#include "detect.h"

using std::this_thread::sleep_for;
using std::chrono::seconds;

int main()
{
    std::ofstream myfile("Logs.txt");
    if(!myfile.is_open()){
      cout << "File open failed! Ending program." << endl;
      return 0;
    }

    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_any_connection(PORT_PATH);

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
    auto offboard = Offboard{system.value()};
    auto telemetry = Telemetry{system.value()};
    auto param = mavsdk::Param{system.value()};

    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        myfile << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
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
                telemetry.position().relative_altitude_m,
                telemetry.heading};
        offboard.set_position_global(currLocation);
        sleep_for(400ms);
    }

    // auto in_air_promise = std::promise<void>{};
    // auto in_air_future = in_air_promise.get_future();
    // Telemetry::LandedStateHandle handle = telemetry.subscribe_landed_state(
    //     [&telemetry, &in_air_promise, &handle](Telemetry::LandedState state) {
    //         if (state == Telemetry::LandedState::InAir) {
    //             telemetry.unsubscribe_landed_state(handle);
    //             in_air_promise.set_value();
    //         }
    //     });
    
    Agent agent(
        action,
        offboard,
        telemetry,
        param,
        myfile);


    agent.getScanContext().positions = {
        {40.0936649, -83.1961460, 24.384},
        {40.0935812, -83.1974774, 24.384},
    }; 

    agent.initTargets("config.txt");

    // agent.start();

    telemetry.subscribe_position([&](Telemetry::Position position) {

        if (telemetry.flight_mode()  == Telemetry::FlightMode::Offboard) {
            agent.start();
        } 
    });




}
