#include <iostream>
#include <chrono>
#include <thread>
#include <future>
#include <cmath>
#include <math.h>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/param/param.h>

#define PORT_PATH "udp://:14540"

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

double targetLatitude = 0;
double targetLongitude = 0;
double currentLatitude = 0;
double currentLongitude = 0;

enum class State {
    INITIAL,
    WAYPOINT1,
    WAYPOINT2,
    WAYPOINT3
};

enum class ReverseState {
    WAYPOINT1,
    WAYPOINT2,
    WAYPOINT3,
    WAYPOINT4,
    HOME,
    STOP,
    FINISHED
};

double distance(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude) {
   double distance = abs(sqrt(pow(currLatitude - targetLatitude,2) + pow(currLongitude - targetLongitude,2)));
   return distance; 
}

float yaw(double currLatitude, double currLongitude, double targetLatitude, double targetLongitude) {
    double position;
    if ((targetLongitude - currLongitude) == 0) {
        position = 0;
    }
    position = (targetLatitude - currLatitude) / (targetLongitude - currLongitude);
    return (float) atan(position) * (180/M_PI);
}

void move_to_next_waypoint(Offboard& offboard, Telemetry& telemetry, State& current_state, Offboard::Result& offboard_result, 
double targetLatitude, double targetLongitude, double coordinates[])
{

    switch (current_state) {
        case State::INITIAL: {
            std::cout << "Reading home position in Global coordinates\n";
            std::cout << "Home Latitude: " << telemetry.position().latitude_deg
          << " Home Longitude: " << telemetry.position().longitude_deg << "\n";
            const auto res_and_gps_origin = telemetry.get_gps_global_origin();
            if (res_and_gps_origin.first != Telemetry::Result::Success) {
            std::cerr << "Telemetry failed: " << res_and_gps_origin.first << '\n';
            }
            sleep_for(seconds(5));
            //Waypoint1
            targetLatitude = coordinates[0];
            targetLongitude = coordinates[1];
            const Offboard::PositionGlobalYaw waypoint1{
                targetLatitude,
                targetLongitude,
                15.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude), 
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint1);
            offboard_result = offboard.start();
            if (offboard_result != Offboard::Result::Success) {
                std::cerr << "Offboard start failed: " << offboard_result << '\n';
            }
            std::cout << "Offboard started\n";
            std::cout << "Going to waypoint1 at 15m relative altitude\n";
            sleep_for(seconds(5));
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                current_state = State::WAYPOINT1;
            } 
        } break;

        case State::WAYPOINT1: {
            std::cout << "Made it to waypoint1\n";
            sleep_for(seconds(5));
            //Waypoint2 (East)
            targetLatitude = coordinates[2];
            targetLongitude = coordinates[3];
            const Offboard::PositionGlobalYaw waypoint2{
                targetLatitude,
                targetLongitude,
                15.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint2);
            std::cout << "Going to waypoint2 at 15m relative altitude\n";
            sleep_for(seconds(5));
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                current_state = State::WAYPOINT2;
            }
        } break;

        case State::WAYPOINT2: {
            std::cout << "Made it to waypoint2\n";
            sleep_for(seconds(5));
            //Waypoint3
            targetLatitude = coordinates[4];
            targetLongitude = coordinates[5];
            const Offboard::PositionGlobalYaw waypoint3{
                targetLatitude,
                targetLongitude,
                15.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint3);
            std::cout << "Going to waypoint3 at 15m relative altitude\n";
            sleep_for(seconds(5));
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                current_state = State::WAYPOINT3;
            }
        } break;

        case State::WAYPOINT3: {
            std::cout << "Made it to waypoint3\n";
            sleep_for(seconds(5));
            //Waypoint4
            targetLatitude = coordinates[6];
            targetLongitude = coordinates[7];
            const Offboard::PositionGlobalYaw waypoint4{
                targetLatitude,
                targetLongitude,
                15.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint4);
            std::cout << "Going to waypoint4 at 15m relative altitude\n";
            sleep_for(seconds(5));
            //if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                //current_state = State::WAYPOINT4;
            //}
        } break;
}
}

void reverse_move_to_next_waypoint(Offboard& offboard, Telemetry& telemetry, ReverseState& reverse_state, Offboard::Result& offboard_result, 
double originLatitude, double originLongitude, double targetLatitude, double targetLongitude, double coordinates[])
{

    switch (reverse_state) {

        case ReverseState::WAYPOINT4: {
            std::cout << "Made it to waypoint4\n";
            std::cout << "Reversing order\n";
            sleep_for(seconds(5));
            //Home
            targetLatitude = coordinates[4];
            targetLongitude = coordinates[5];
            const Offboard::PositionGlobalYaw waypoint3{
                targetLatitude,
                targetLongitude,
                15.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint3);
            std::cout << "Going to waypoint3 at 15m relative altitude\n";
            sleep_for(seconds(5));
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                reverse_state = ReverseState::WAYPOINT3;
            }
        } break;

        case ReverseState::WAYPOINT3: {
            std::cout << "Made it to waypoint3\n";
            sleep_for(seconds(5));
            //Waypoint4
            targetLatitude = coordinates[2];
            targetLongitude = coordinates[3];
            const Offboard::PositionGlobalYaw waypoint2{
                targetLatitude,
                targetLongitude,
                15.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint2);
            std::cout << "Going to waypoint2 at 15m relative altitude\n";
            sleep_for(seconds(5));
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                reverse_state = ReverseState::WAYPOINT1;
            }
        } break;

        case ReverseState::WAYPOINT2: {
            std::cout << "Made it to waypoint2\n";
            sleep_for(seconds(5));
            //Waypoint3
            targetLatitude = coordinates[0];
            targetLongitude = coordinates[1];
            const Offboard::PositionGlobalYaw waypoint1{
                targetLatitude,
                targetLongitude,
                15.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(waypoint1);
            std::cout << "Going to waypoint3 at 15m relative altitude\n";
            sleep_for(seconds(5));
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                reverse_state = ReverseState::WAYPOINT1;
            }
        } break;

        case ReverseState::WAYPOINT1: {
            std::cout << "Made it to waypoint1\n";
            sleep_for(seconds(5));
            //Home
            targetLatitude = originLatitude;
            targetLongitude = originLongitude;
            const Offboard::PositionGlobalYaw home{
                targetLatitude,
                targetLongitude,
                15.0f,
                yaw(telemetry.position().latitude_deg, telemetry.position().longitude_deg, targetLatitude, targetLongitude),
                Offboard::PositionGlobalYaw::AltitudeType::RelHome};
            offboard.set_position_global(home);
            std::cout << "Going home at 15m relative altitude\n";
            sleep_for(seconds(5));
            if (distance(telemetry.position().latitude_deg,telemetry.position().longitude_deg,targetLatitude,targetLongitude) <= 0.00002) {
                reverse_state = ReverseState::HOME;
            }
        } break;

        case ReverseState::HOME: {
            std::cout << "Made it home\n";
            sleep_for(seconds(5));
            // Stop offboard mode
            offboard_result = offboard.stop();
            std::cout << "Stopping Offboard\n";
            if (offboard_result != Offboard::Result::Success) {
            std::cerr << "Offboard stop failed: " << offboard_result << '\n';
            }
            reverse_state = ReverseState::STOP;
        } break;

        case ReverseState::STOP: {
            std::cout << "Offboard stopped\n";
            reverse_state = ReverseState::FINISHED;
        } break;

        case ReverseState::FINISHED: {
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
    auto param = Param{system.value()};

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

    const auto set_velocity = param.set_param_float("MPC_XY_VEL_MAX", 5.0);
    if (set_velocity != Param::Result::Success) {
        std::cerr << "Velocity not set: " << set_velocity << '\n';
        return 1;
    }

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
    //State previous_state = State::INITIAL;
    Offboard::Result offboard_result;
    double originLatitude = telemetry.position().latitude_deg;
    double originLongitude = telemetry.position().longitude_deg;
    double coordinates[] = {telemetry.position().latitude_deg + 0.0001, telemetry.position().longitude_deg, telemetry.position().latitude_deg, 
    telemetry.position().longitude_deg + 0.0001,telemetry.position().latitude_deg + 0.0002, telemetry.position().longitude_deg + 0.0001,
    telemetry.position().latitude_deg + 0.0003, telemetry.position().longitude_deg + 0.0002};

    while (current_state != State::WAYPOINT4) {
        move_to_next_waypoint(offboard, telemetry, current_state, offboard_result, targetLatitude, targetLongitude, coordinates);
    }

    ReverseState reverse_state = ReverseState::WAYPOINT4;
    while (reverse_state != ReverseState::FINISHED) {
        reverse_move_to_next_waypoint(offboard, telemetry, reverse_state, offboard_result, originLatitude, originLongitude, targetLatitude, targetLongitude, coordinates);
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
