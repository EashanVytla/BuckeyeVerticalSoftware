#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>
#include <fstream>

#include "agent.h"
// #include "detect.h"

#define PHYSICAL_PORT_PATH "serial:///dev/ttyTHS1"
#define SIM_PORT_PATH  "udp://:14540"


#define TEN_METERS_APPROX 0.0001

using std::this_thread::sleep_for;
using std::chrono::seconds;

int main()
{

    std::ofstream myfile;
    myfile.open("logs.txt");

    if(!myfile.is_open()){
        std::cout << "File open failed! Ending program." << std::endl;
        return 0;
    }


    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_any_connection(SIM_PORT_PATH);

    if (connection_result != ConnectionResult::Success) {
        std::cerr << "Connection failed: " << connection_result << '\n';
        return 1;
    }

    auto system = mavsdk.first_autopilot(3.0);
    if (!system) {
        std::cerr << "Timed out waiting for system\n";
        return 1;
    }

   
    auto action = Action{system.value()};
    auto offboard = Offboard{system.value()};
    auto telemetry = Telemetry{system.value()};
    auto param = mavsdk::Param{system.value()};

    // std::cout << "Here " << telemetry.position() << std::endl << std::endl;



    while (!telemetry.health_all_ok()) {
        std::cout << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }
    std::cout << "System is ready\n";



    const auto arm_result = action.arm();
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


    in_air_future.wait_for(seconds(10));
    if (in_air_future.wait_for(seconds(3)) == std::future_status::timeout) {
        std::cerr << "Takeoff timed out.\n";
        return 1;
    }



    const auto set_velocity = param.set_param_float("MPC_XY_VEL_MAX", 5.0);
    if (set_velocity != mavsdk::Param::Result::Success) {
        std::cerr << "Velocity not set: " << set_velocity << '\n';
        myfile << "Velocity not set: " << set_velocity << '\n';
        return 1;
    }


    
    // while(telemetry.flight_mode() != Telemetry::FlightMode::Offboard) {
    //     std::cout << "Waiting for Offboard\n";
    //     myfile << "Waiting for Offboard\n";
    //     const Offboard::PositionGlobalYaw currLocation{
    //             telemetry.position().latitude_deg,
    //             telemetry.position().longitude_deg,
    //             7.0f,
    //             0.0f};
    //     offboard.set_position_global(currLocation);
    //     sleep_for(400ms);
    // }


    Agent agent(
        action,
        offboard,
        telemetry,
        param,
        myfile);


    // agent.getScanContext().positions = {
    //     {40.0979633, -83.1982978, 24.384},
    //     {40.0979265, -83.1985653, 24.384},
    //     {40.0978664, -83.1987608, 24.384},
    //     {40.0978334, -83.1989082, 24.384},
    //     {40.0978456, -83.1990924, 24.384},
    //     {40.0978456, -83.1990924, 24.384},
    // }; 


    agent.getScanContext().positions = {
        {telemetry.position().latitude_deg, telemetry.position().longitude_deg, 24.384},
        {telemetry.position().latitude_deg + TEN_METERS_APPROX, telemetry.position().longitude_deg, 24.384},
        {telemetry.position().latitude_deg + TEN_METERS_APPROX, telemetry.position().longitude_deg + TEN_METERS_APPROX, 24.384},
        {telemetry.position().latitude_deg, telemetry.position().longitude_deg + TEN_METERS_APPROX, 24.384},
        {telemetry.position().latitude_deg, telemetry.position().longitude_deg, 24.384}
    };


    cout << "in main for agent" << endl;

    for (int i = 0; i < agent.getScanContext().positions.size(); i++) {
        Coordinate c = agent.getScanContext().positions.at(i);
        cout << c.latitude  << ", " << c.longitude  << endl;
    }
    // cout << agent.getScanContext().positions << endl; 

    agent.initTargets("config.txt");


    cout << "actually starting the agent" << endl;

    agent.start();


    auto offboard_result = offboard.start();
    
    while (agent.isRunning())
        sleep_for(1000ms);

    return 0;


    // const auto land_result = action.land();
    // if (land_result != Action::Result::Success) {
    //     std::cerr << "Landing failed: " << land_result << '\n';
    //     return 1;
    // }

    // while (telemetry.in_air()) {
    //     std::cout << "Vehicle is landing...\n";
    //     sleep_for(seconds(1));
    // }
    // std::cout << "Landed!\n";

    // sleep_for(seconds(3));
    // std::cout << "Finished...\n";

    // return 0;










    // auto in_air_promise = std::promise<void>{};
    // auto in_air_future = in_air_promise.get_future();
    // Telemetry::LandedStateHandle handle = telemetry.subscribe_landed_state(
    //     [&telemetry, &in_air_promise, &handle](Telemetry::LandedState state) {
    //         if (state == Telemetry::LandedState::InAir) {
    //             telemetry.unsubscribe_landed_state(handle);
    //             in_air_promise.set_value();
    //         }
    //     });
    



    // agent.start();

    // telemetry.subscribe_position([&](Telemetry::Position position) {

    //     if (telemetry.flight_mode()  == Telemetry::FlightMode::Offboard) {
    //         agent.start();
    //     } 
    // });




}