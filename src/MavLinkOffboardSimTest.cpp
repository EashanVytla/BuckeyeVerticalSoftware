#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#define PORT_PATH "udp://localhost:14540"

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

enum class State {
    INITIAL,
    WAYPOINT1,
    WAYPOINT2,
    HOME,
    STOP,
    FINISHED
};

double distance(double latitude, double longitude) {
   double distance = abs(sqrt(pow(latitude,2) + pow(longitude,2)));
   return distance; 
}

void move_to_next_waypoint(Offboard& offboard, Telemetry& telemetry, State& current_state)
{
    switch (current_state) {
        case State::INITIAL: {
            std::cout << "Moving to initial position\n";
            // Set the first waypoint (North)
            const Offboard::PositionGlobalYaw waypoint1{
                telemetry.position().latitude_deg + 0.0001,
                telemetry.position().longitude_deg,
                20.0f,
                0.0f};
            offboard.set_position_global(waypoint1);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg <= 0.00002)) {
                current_state = State::WAYPOINT1;
            } 
        } break;

        case State::WAYPOINT1: {
            std::cout << "Going North at 20m relative altitude\n";
            sleep_for(seconds(10));
            // Set the second waypoint (East)
            const Offboard::PositionGlobalYaw waypoint2{
                telemetry.position().latitude_deg + 0.0001,
                telemetry.position().longitude_deg + 0.0001,
                15.0f,
                90.0f,
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint2);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg <= 0.00002)) {
                current_state = State::WAYPOINT2;
            }
        } break;

        case State::WAYPOINT2: {
            std::cout << "Going East at 15m relative altitude\n";
            sleep_for(seconds(10));
            // Set the third waypoint (Home)
            const Offboard::PositionGlobalYaw home{
                telemetry.position().latitude_deg,
                telemetry.position().longitude_deg,
                telemetry.position().absolute_altitude_m + 10.0f,
                180.0f,
                Offboard::PositionGlobalYaw::AltitudeType::Amsl};
            offboard.set_position_global(home);
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg <= 0.00002)) {
                current_state = State::HOME;
            }
        } break;

        case State::HOME: {
            std::cout << "Going Home facing south at " << (telemetry.position().absolute_altitude_m + 10.0f)
                      << "m AMSL altitude\n";
            sleep_for(seconds(10));
            // Stop offboard mode
            offboard.stop();
            current_state = State::STOP;
        } break;

        case State::STOP: {
            std::cout << "Offboard stopped\n";
            current_state = State::FINISHED;
        } break;

        case State::FINISHED: {
            std::cout << "Finished\n";
        } break;
    }
}

int main()
{
    Mavsdk mavsdk{Mavsdk::Configuration{Mavsdk::ComponentType::CompanionComputer}};
    ConnectionResult connection_result = mavsdk.add_any_connection(PORT_PATH);

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

    State current_state = State::INITIAL;

    while (current_state != State::FINISHED) {
        move_to_next_waypoint(offboard, telemetry, current_state);
    }

    const auto land_result = action.land();
    if (land_result != Action::Result::Success) {
        std::cerr << "Landing failed: " << land_result << '\n';
        return 1;
    }

    while (telemetry.in_air()) {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(1));
    }
    std::cout << "Landed!\n";

    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}
